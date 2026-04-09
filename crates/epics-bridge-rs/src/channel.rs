//! BridgeChannel: single-record PVA channel.
//!
//! Corresponds to C++ QSRV's `PDBSinglePV` / `PDBSingleChannel`.

use std::sync::Arc;

use epics_base_rs::server::database::PvDatabase;
use epics_base_rs::types::DbFieldType;
use epics_pva_rs::pvdata::{FieldDesc, PvStructure};

use crate::convert::dbf_to_scalar_type;
use crate::error::{BridgeError, BridgeResult};
use crate::monitor::BridgeMonitor;
use crate::provider::Channel;
use crate::pvif::{
    NtType, build_field_desc_for_nt, pv_structure_to_epics, snapshot_to_pv_structure,
};

/// A PVA channel backed by a single EPICS database record.
pub struct BridgeChannel {
    db: Arc<PvDatabase>,
    record_name: String,
    nt_type: NtType,
    /// The DBF type of the primary value field.
    value_dbf: DbFieldType,
}

impl BridgeChannel {
    /// Create a new channel for a record.
    ///
    /// Reads the record type to determine the NormativeType mapping.
    pub async fn new(db: Arc<PvDatabase>, name: &str) -> BridgeResult<Self> {
        let (record_name, _field) = epics_base_rs::server::database::parse_pv_name(name);

        let rec = db
            .get_record(record_name)
            .await
            .ok_or_else(|| BridgeError::RecordNotFound(record_name.to_string()))?;

        let instance = rec.read().await;
        let rtyp = instance.record.record_type();
        let nt_type = NtType::from_record_type(rtyp);

        // Determine the DBF type of the primary value field
        let value_dbf = instance
            .record
            .field_list()
            .iter()
            .find(|f| f.name == "VAL")
            .map(|f| f.dbf_type)
            .unwrap_or(DbFieldType::Double);

        Ok(Self {
            db,
            record_name: record_name.to_string(),
            nt_type,
            value_dbf,
        })
    }

    /// The NormativeType for this channel.
    pub fn nt_type(&self) -> NtType {
        self.nt_type
    }
}

impl Channel for BridgeChannel {
    fn channel_name(&self) -> &str {
        &self.record_name
    }

    async fn get(&self, _request: &PvStructure) -> BridgeResult<PvStructure> {
        let rec = self
            .db
            .get_record(&self.record_name)
            .await
            .ok_or_else(|| BridgeError::RecordNotFound(self.record_name.clone()))?;

        let instance = rec.read().await;
        let snapshot = instance.snapshot_for_field("VAL").ok_or_else(|| {
            BridgeError::FieldNotFound {
                record: self.record_name.clone(),
                field: "VAL".into(),
            }
        })?;

        Ok(snapshot_to_pv_structure(&snapshot, self.nt_type))
    }

    async fn put(&self, value: &PvStructure) -> BridgeResult<()> {
        let epics_val =
            pv_structure_to_epics(value).ok_or_else(|| BridgeError::TypeMismatch {
                expected: "extractable value".into(),
                got: format!("{}", value.struct_id),
            })?;

        self.db
            .put_record_field_from_ca(&self.record_name, "VAL", epics_val)
            .await
            .map_err(|e| BridgeError::PutRejected(e.to_string()))?;

        Ok(())
    }

    async fn get_field(&self) -> BridgeResult<FieldDesc> {
        let scalar_type = dbf_to_scalar_type(self.value_dbf);
        Ok(build_field_desc_for_nt(self.nt_type, scalar_type))
    }

    async fn create_monitor(&self) -> BridgeResult<BridgeMonitor> {
        Ok(BridgeMonitor::new(
            self.db.clone(),
            self.record_name.clone(),
            self.nt_type,
        ))
    }
}
