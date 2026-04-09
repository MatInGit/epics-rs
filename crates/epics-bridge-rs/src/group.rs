//! GroupChannel: multi-record composite PVA channel.
//!
//! Corresponds to C++ QSRV's `PDBGroupPV` / `PDBGroupChannel`.
//! A group PV combines fields from multiple EPICS database records
//! into a single PvStructure.

use std::sync::Arc;

use epics_base_rs::server::database::PvDatabase;
use epics_pva_rs::pvdata::{FieldDesc, PvField, PvStructure, ScalarType};

use crate::convert::epics_to_pv_field;
use crate::error::{BridgeError, BridgeResult};
use crate::group_config::{GroupMember, GroupPvDef};
use crate::monitor::BridgeMonitor;
use crate::provider::Channel;
use crate::pvif::{self, FieldMapping, NtType};

/// A PVA channel backed by a group of EPICS database records.
pub struct GroupChannel {
    db: Arc<PvDatabase>,
    def: GroupPvDef,
}

impl GroupChannel {
    pub fn new(db: Arc<PvDatabase>, def: GroupPvDef) -> Self {
        Self { db, def }
    }

    /// Read all member values and compose into a single PvStructure.
    async fn read_group(&self) -> BridgeResult<PvStructure> {
        let struct_id = self.def.struct_id.as_deref().unwrap_or("structure");
        let mut pv = PvStructure::new(struct_id);

        for member in &self.def.members {
            if member.mapping == FieldMapping::Proc {
                continue;
            }

            let field = self.read_member(member).await?;
            pv.fields.push((member.field_name.clone(), field));
        }

        Ok(pv)
    }

    /// Read a single member's value from the database.
    async fn read_member(&self, member: &GroupMember) -> BridgeResult<PvField> {
        let (record_name, field_name) =
            epics_base_rs::server::database::parse_pv_name(&member.channel);

        let rec = self
            .db
            .get_record(record_name)
            .await
            .ok_or_else(|| BridgeError::RecordNotFound(record_name.to_string()))?;

        let instance = rec.read().await;

        match member.mapping {
            FieldMapping::Scalar => {
                // Full NTScalar mapping with metadata
                let snapshot = instance.snapshot_for_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                let rtyp = instance.record.record_type();
                let nt_type = NtType::from_record_type(rtyp);
                Ok(PvField::Structure(pvif::snapshot_to_pv_structure(
                    &snapshot, nt_type,
                )))
            }
            FieldMapping::Plain => {
                // Value only, no metadata
                let value = instance.resolve_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                Ok(epics_to_pv_field(&value))
            }
            FieldMapping::Meta => {
                // Alarm + timestamp only
                let snapshot = instance.snapshot_for_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                let mut meta = PvStructure::new("meta_t");
                meta.fields.push((
                    "alarm".into(),
                    PvField::Structure(build_alarm_from_snapshot(&snapshot)),
                ));
                meta.fields.push((
                    "timeStamp".into(),
                    PvField::Structure(build_timestamp_from_snapshot(&snapshot)),
                ));
                Ok(PvField::Structure(meta))
            }
            FieldMapping::Any => {
                // Wrap value in variant-like structure
                let value = instance.resolve_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                Ok(epics_to_pv_field(&value))
            }
            FieldMapping::Proc => {
                // Process-only: no value to read
                Ok(PvField::Scalar(epics_pva_rs::pvdata::ScalarValue::Int(0)))
            }
        }
    }
}

impl Channel for GroupChannel {
    fn channel_name(&self) -> &str {
        &self.def.name
    }

    async fn get(&self, _request: &PvStructure) -> BridgeResult<PvStructure> {
        self.read_group().await
    }

    async fn put(&self, value: &PvStructure) -> BridgeResult<()> {
        // Put members in put_order sequence
        let mut ordered: Vec<&GroupMember> = self.def.members.iter().collect();
        ordered.sort_by_key(|m| m.put_order);

        for member in ordered {
            let pv_field = match value.get_field(&member.field_name) {
                Some(f) => f,
                None => continue, // Field not included in put request
            };

            let epics_val = match crate::convert::pv_field_to_epics(pv_field) {
                Some(v) => v,
                None => continue, // Can't convert (e.g., structure fields)
            };

            let (record_name, field_name) =
                epics_base_rs::server::database::parse_pv_name(&member.channel);

            if member.mapping == FieldMapping::Proc {
                // Process-only: trigger record processing without writing a value
                self.db
                    .process_record(record_name)
                    .await
                    .map_err(|e| BridgeError::PutRejected(e.to_string()))?;
            } else {
                self.db
                    .put_record_field_from_ca(record_name, field_name, epics_val)
                    .await
                    .map_err(|e| BridgeError::PutRejected(e.to_string()))?;
            }
        }

        Ok(())
    }

    async fn get_field(&self) -> BridgeResult<FieldDesc> {
        let struct_id = self.def.struct_id.as_deref().unwrap_or("structure");
        let mut fields = Vec::new();

        for member in &self.def.members {
            if member.mapping == FieldMapping::Proc {
                continue;
            }

            // For now, use Double as default scalar type for group members.
            // A full implementation would introspect each record's field type.
            let desc = match member.mapping {
                FieldMapping::Scalar => {
                    pvif::build_field_desc_for_nt(NtType::Scalar, ScalarType::Double)
                }
                FieldMapping::Plain => FieldDesc::Scalar(ScalarType::Double),
                FieldMapping::Meta => FieldDesc::Structure {
                    struct_id: "meta_t".into(),
                    fields: vec![
                        (
                            "alarm".into(),
                            FieldDesc::Structure {
                                struct_id: "alarm_t".into(),
                                fields: vec![
                                    ("severity".into(), FieldDesc::Scalar(ScalarType::Int)),
                                    ("status".into(), FieldDesc::Scalar(ScalarType::Int)),
                                    ("message".into(), FieldDesc::Scalar(ScalarType::String)),
                                ],
                            },
                        ),
                        (
                            "timeStamp".into(),
                            FieldDesc::Structure {
                                struct_id: "time_t".into(),
                                fields: vec![
                                    (
                                        "secondsPastEpoch".into(),
                                        FieldDesc::Scalar(ScalarType::Long),
                                    ),
                                    ("nanoseconds".into(), FieldDesc::Scalar(ScalarType::Int)),
                                    ("userTag".into(), FieldDesc::Scalar(ScalarType::Int)),
                                ],
                            },
                        ),
                    ],
                },
                FieldMapping::Any | FieldMapping::Proc => FieldDesc::Scalar(ScalarType::Double),
            };

            fields.push((member.field_name.clone(), desc));
        }

        Ok(FieldDesc::Structure {
            struct_id: struct_id.into(),
            fields,
        })
    }

    async fn create_monitor(&self) -> BridgeResult<BridgeMonitor> {
        // Group monitor: subscribe to the first member's channel.
        // A full implementation would subscribe to all members and merge
        // updates according to trigger rules.
        let first_member = self.def.members.first().ok_or_else(|| {
            BridgeError::GroupConfigError("group has no members".into())
        })?;

        let (record_name, _) =
            epics_base_rs::server::database::parse_pv_name(&first_member.channel);

        Ok(BridgeMonitor::new(
            self.db.clone(),
            record_name.to_string(),
            NtType::Scalar,
        ))
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn build_alarm_from_snapshot(
    snapshot: &epics_base_rs::server::snapshot::Snapshot,
) -> PvStructure {
    use epics_pva_rs::pvdata::ScalarValue;
    let mut alarm = PvStructure::new("alarm_t");
    alarm.fields.push((
        "severity".into(),
        PvField::Scalar(ScalarValue::Int(snapshot.alarm.severity as i32)),
    ));
    alarm.fields.push((
        "status".into(),
        PvField::Scalar(ScalarValue::Int(snapshot.alarm.status as i32)),
    ));
    alarm.fields.push((
        "message".into(),
        PvField::Scalar(ScalarValue::String(String::new())),
    ));
    alarm
}

fn build_timestamp_from_snapshot(
    snapshot: &epics_base_rs::server::snapshot::Snapshot,
) -> PvStructure {
    use epics_pva_rs::pvdata::ScalarValue;
    use std::time::UNIX_EPOCH;

    let mut ts = PvStructure::new("time_t");
    let (secs, nanos) = match snapshot.timestamp.duration_since(UNIX_EPOCH) {
        Ok(d) => (d.as_secs() as i64, d.subsec_nanos() as i32),
        Err(_) => (0, 0),
    };
    ts.fields
        .push(("secondsPastEpoch".into(), PvField::Scalar(ScalarValue::Long(secs))));
    ts.fields
        .push(("nanoseconds".into(), PvField::Scalar(ScalarValue::Int(nanos))));
    ts.fields
        .push(("userTag".into(), PvField::Scalar(ScalarValue::Int(0))));
    ts
}
