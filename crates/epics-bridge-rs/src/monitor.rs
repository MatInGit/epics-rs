//! BridgeMonitor: bridges DbSubscription to PVA monitor.
//!
//! Corresponds to C++ QSRV's `PDBSingleMonitor` / `BaseMonitor`.
//!
//! Key difference from C++: Rust uses `mpsc::Receiver` as the built-in
//! queue, so we don't need the C++ BaseMonitor's empty/inuse deque +
//! overflow BitSet pattern.

use std::sync::Arc;

use epics_base_rs::server::database::db_access::DbSubscription;
use epics_base_rs::server::database::PvDatabase;
use epics_pva_rs::pvdata::PvStructure;

use crate::error::{BridgeError, BridgeResult};
use crate::provider::PvaMonitor;
use crate::pvif::{NtType, snapshot_to_pv_structure};

/// A PVA monitor backed by a DbSubscription.
pub struct BridgeMonitor {
    db: Arc<PvDatabase>,
    record_name: String,
    nt_type: NtType,
    subscription: Option<DbSubscription>,
    running: bool,
}

impl BridgeMonitor {
    pub fn new(db: Arc<PvDatabase>, record_name: String, nt_type: NtType) -> Self {
        Self {
            db,
            record_name,
            nt_type,
            subscription: None,
            running: false,
        }
    }
}

impl PvaMonitor for BridgeMonitor {
    async fn start(&mut self) -> BridgeResult<()> {
        if self.running {
            return Ok(());
        }

        let sub = DbSubscription::subscribe(&self.db, &self.record_name)
            .await
            .ok_or_else(|| BridgeError::RecordNotFound(self.record_name.clone()))?;

        self.subscription = Some(sub);
        self.running = true;
        Ok(())
    }

    async fn poll(&mut self) -> Option<PvStructure> {
        let sub = self.subscription.as_mut()?;

        // Wait for the next value change
        let value = sub.recv().await?;

        // Build a minimal snapshot from the received value and convert
        // to the appropriate NormativeType structure.
        //
        // Note: DbSubscription.recv() returns EpicsValue only, not a full
        // Snapshot with alarm/display metadata. For a complete implementation,
        // we would need to re-read the record's snapshot_for_field() here.
        // For now, we build a minimal snapshot.
        let snapshot = epics_base_rs::server::snapshot::Snapshot::new(
            value,
            0, // alarm status
            0, // alarm severity
            std::time::SystemTime::now(),
        );

        Some(snapshot_to_pv_structure(&snapshot, self.nt_type))
    }

    async fn stop(&mut self) {
        self.subscription = None;
        self.running = false;
    }
}
