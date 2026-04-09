//! GroupChannel and GroupMonitor: multi-record composite PVA channel.
//!
//! Corresponds to C++ QSRV's `PDBGroupPV` / `PDBGroupChannel` / `PDBGroupMonitor`.
//! A group PV combines fields from multiple EPICS database records
//! into a single PvStructure.

use std::sync::Arc;

use epics_base_rs::server::database::db_access::DbSubscription;
use epics_base_rs::server::database::PvDatabase;
use epics_base_rs::types::DbFieldType;
use epics_pva_rs::pvdata::{FieldDesc, PvField, PvStructure, ScalarType};

use crate::convert::{dbf_to_scalar_type, epics_to_pv_field};
use crate::error::{BridgeError, BridgeResult};
use crate::group_config::{GroupMember, GroupPvDef, TriggerDef};
use crate::monitor::BridgeMonitor;
use crate::pvif::{self, FieldMapping, NtType};

// ---------------------------------------------------------------------------
// GroupChannel
// ---------------------------------------------------------------------------

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
    pub(crate) async fn read_group(&self) -> BridgeResult<PvStructure> {
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
                let value = instance.resolve_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                Ok(epics_to_pv_field(&value))
            }
            FieldMapping::Meta => {
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
                let value = instance.resolve_field(field_name).ok_or_else(|| {
                    BridgeError::FieldNotFound {
                        record: record_name.to_string(),
                        field: field_name.to_string(),
                    }
                })?;
                Ok(epics_to_pv_field(&value))
            }
            FieldMapping::Proc => {
                Ok(PvField::Scalar(epics_pva_rs::pvdata::ScalarValue::Int(0)))
            }
        }
    }

    /// Introspect a member's actual DBF type and record type from the database.
    async fn introspect_member(
        &self,
        member: &GroupMember,
    ) -> BridgeResult<(NtType, ScalarType)> {
        let (record_name, field_name) =
            epics_base_rs::server::database::parse_pv_name(&member.channel);

        let rec = self
            .db
            .get_record(record_name)
            .await
            .ok_or_else(|| BridgeError::RecordNotFound(record_name.to_string()))?;

        let instance = rec.read().await;
        let rtyp = instance.record.record_type();
        let nt_type = NtType::from_record_type(rtyp);

        let field_upper = field_name.to_ascii_uppercase();
        let value_dbf = instance
            .record
            .field_list()
            .iter()
            .find(|f| f.name == field_upper)
            .map(|f| f.dbf_type)
            .unwrap_or(DbFieldType::Double);

        Ok((nt_type, dbf_to_scalar_type(value_dbf)))
    }
}

impl crate::provider::Channel for GroupChannel {
    fn channel_name(&self) -> &str {
        &self.def.name
    }

    async fn get(&self, _request: &PvStructure) -> BridgeResult<PvStructure> {
        self.read_group().await
    }

    async fn put(&self, value: &PvStructure) -> BridgeResult<()> {
        let mut ordered: Vec<&GroupMember> = self.def.members.iter().collect();
        ordered.sort_by_key(|m| m.put_order);

        for member in ordered {
            let pv_field = match value.get_field(&member.field_name) {
                Some(f) => f,
                None => continue,
            };

            let (record_name, field_name) =
                epics_base_rs::server::database::parse_pv_name(&member.channel);

            if member.mapping == FieldMapping::Proc {
                self.db
                    .process_record(record_name)
                    .await
                    .map_err(|e| BridgeError::PutRejected(e.to_string()))?;
            } else {
                let epics_val = match crate::convert::pv_field_to_epics(pv_field) {
                    Some(v) => v,
                    None => continue,
                };
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

            // Introspect actual record field type (not hardcoded Double)
            let (nt_type, scalar_type) = self.introspect_member(member).await?;

            let desc = match member.mapping {
                FieldMapping::Scalar => pvif::build_field_desc_for_nt(nt_type, scalar_type),
                FieldMapping::Plain => FieldDesc::Scalar(scalar_type),
                FieldMapping::Meta => meta_desc(),
                FieldMapping::Any => FieldDesc::Scalar(scalar_type),
                FieldMapping::Proc => continue,
            };

            fields.push((member.field_name.clone(), desc));
        }

        Ok(FieldDesc::Structure {
            struct_id: struct_id.into(),
            fields,
        })
    }

    async fn create_monitor(&self) -> BridgeResult<AnyMonitor> {
        Ok(AnyMonitor::Group(GroupMonitor::new(
            self.db.clone(),
            self.def.clone(),
        )))
    }
}

// ---------------------------------------------------------------------------
// GroupMonitor
// ---------------------------------------------------------------------------

/// A PVA monitor for a group PV that subscribes to all member records.
///
/// Corresponds to C++ QSRV's `PDBGroupMonitor` + `pdb_group_event()`.
/// Subscribes to all members and evaluates trigger rules to determine
/// which fields to include in each update.
pub struct GroupMonitor {
    db: Arc<PvDatabase>,
    def: GroupPvDef,
    /// Per-member subscriptions: (member_index, subscription)
    subscriptions: Vec<(usize, DbSubscription)>,
    running: bool,
    /// Initial complete group snapshot (sent on first poll)
    initial_snapshot: Option<PvStructure>,
}

impl GroupMonitor {
    pub fn new(db: Arc<PvDatabase>, def: GroupPvDef) -> Self {
        Self {
            db,
            def,
            subscriptions: Vec::new(),
            running: false,
            initial_snapshot: None,
        }
    }

    /// Check if a trigger should cause a group update.
    fn should_notify(trigger: &TriggerDef) -> bool {
        !matches!(trigger, TriggerDef::None)
    }
}

impl crate::provider::PvaMonitor for GroupMonitor {
    async fn start(&mut self) -> BridgeResult<()> {
        if self.running {
            return Ok(());
        }

        // Subscribe to all members that have triggers (like C++ pdb.cpp:568-586)
        for (idx, member) in self.def.members.iter().enumerate() {
            if !Self::should_notify(&member.triggers) {
                continue;
            }

            let (record_name, _) =
                epics_base_rs::server::database::parse_pv_name(&member.channel);

            if let Some(sub) = DbSubscription::subscribe(&self.db, record_name).await {
                self.subscriptions.push((idx, sub));
            }
        }

        // Read initial complete group snapshot (like C++ BaseMonitor::connect)
        let group_channel = GroupChannel::new(self.db.clone(), self.def.clone());
        if let Ok(snapshot) = group_channel.read_group().await {
            self.initial_snapshot = Some(snapshot);
        }

        self.running = true;
        Ok(())
    }

    async fn poll(&mut self) -> Option<PvStructure> {
        // Return initial snapshot first (C++ BaseMonitor::connect behavior)
        if let Some(initial) = self.initial_snapshot.take() {
            return Some(initial);
        }

        if self.subscriptions.is_empty() {
            return std::future::pending().await;
        }

        // Wait for any member to change using try_recv poll
        // (A production implementation would use tokio::select! macro,
        //  but that requires a fixed number of branches at compile time.
        //  This polling approach works for the interface design.)
        loop {
            for (idx, sub) in &mut self.subscriptions {
                if let Some(_snapshot) = sub.recv_snapshot().await {
                    // A member changed — evaluate trigger rules
                    let member = &self.def.members[*idx];

                    match &member.triggers {
                        TriggerDef::None => continue,
                        TriggerDef::All | TriggerDef::Fields(_) => {
                            // Re-read the entire group for a consistent composite snapshot.
                            // (C++ does selective field update via trigger indices,
                            //  but full re-read is correct and simpler.)
                            let group_channel =
                                GroupChannel::new(self.db.clone(), self.def.clone());
                            return group_channel.read_group().await.ok();
                        }
                    }
                }
            }
        }
    }

    async fn stop(&mut self) {
        self.subscriptions.clear();
        self.running = false;
        self.initial_snapshot = None;
    }
}

// ---------------------------------------------------------------------------
// AnyMonitor
// ---------------------------------------------------------------------------

/// Enum dispatch for monitor types (single record vs group).
pub enum AnyMonitor {
    Single(BridgeMonitor),
    Group(GroupMonitor),
}

impl crate::provider::PvaMonitor for AnyMonitor {
    async fn poll(&mut self) -> Option<PvStructure> {
        match self {
            Self::Single(m) => m.poll().await,
            Self::Group(m) => m.poll().await,
        }
    }

    async fn start(&mut self) -> BridgeResult<()> {
        match self {
            Self::Single(m) => m.start().await,
            Self::Group(m) => m.start().await,
        }
    }

    async fn stop(&mut self) {
        match self {
            Self::Single(m) => m.stop().await,
            Self::Group(m) => m.stop().await,
        }
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn meta_desc() -> FieldDesc {
    FieldDesc::Structure {
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
                        ("secondsPastEpoch".into(), FieldDesc::Scalar(ScalarType::Long)),
                        ("nanoseconds".into(), FieldDesc::Scalar(ScalarType::Int)),
                        ("userTag".into(), FieldDesc::Scalar(ScalarType::Int)),
                    ],
                },
            ),
        ],
    }
}

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
