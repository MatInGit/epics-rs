//! ChannelProvider trait and BridgeProvider implementation.
//!
//! Corresponds to C++ QSRV's `PDBProvider` (pdb.h/pdb.cpp).
//!
//! The trait definitions here are temporary — they will move to `epics-pva-rs`
//! once the PVA server is implemented by the spvirit maintainer.

use std::collections::HashMap;
use std::sync::Arc;

use epics_base_rs::server::database::PvDatabase;
use epics_pva_rs::pvdata::{FieldDesc, PvStructure};

use crate::channel::BridgeChannel;
use crate::error::{BridgeError, BridgeResult};
use crate::group::GroupChannel;
use crate::group_config::GroupPvDef;

// ---------------------------------------------------------------------------
// Trait definitions (to be moved to epics-pva-rs)
// ---------------------------------------------------------------------------

/// PVA ChannelProvider interface.
///
/// Corresponds to C++ `pva::ChannelProvider`. A PVA server calls into this
/// trait to resolve channel names and create channel instances.
pub trait ChannelProvider: Send + Sync {
    /// Provider name (e.g., "BRIDGE").
    fn provider_name(&self) -> &str;

    /// Check if a channel name exists (for UDP search responses).
    fn channel_find(
        &self,
        name: &str,
    ) -> impl std::future::Future<Output = bool> + Send;

    /// List all available channel names.
    fn channel_list(
        &self,
    ) -> impl std::future::Future<Output = Vec<String>> + Send;

    /// Create a channel for the given name.
    fn create_channel(
        &self,
        name: &str,
    ) -> impl std::future::Future<Output = BridgeResult<AnyChannel>> + Send;
}

/// PVA Channel interface.
///
/// Corresponds to C++ `pva::Channel`. Each instance is bound to a single
/// PV (record or group).
pub trait Channel: Send + Sync {
    /// The channel (PV) name.
    fn channel_name(&self) -> &str;

    /// Get: read current value + metadata as a PvStructure.
    fn get(
        &self,
        request: &PvStructure,
    ) -> impl std::future::Future<Output = BridgeResult<PvStructure>> + Send;

    /// Put: write a PvStructure value into the record.
    fn put(
        &self,
        value: &PvStructure,
    ) -> impl std::future::Future<Output = BridgeResult<()>> + Send;

    /// GetField: return the type description (FieldDesc) for introspection.
    fn get_field(
        &self,
    ) -> impl std::future::Future<Output = BridgeResult<FieldDesc>> + Send;

    /// Create a monitor for this channel.
    fn create_monitor(
        &self,
    ) -> impl std::future::Future<Output = BridgeResult<crate::group::AnyMonitor>> + Send;
}

/// PVA Monitor interface.
///
/// Corresponds to C++ `pva::Monitor` / `BaseMonitor`.
pub trait PvaMonitor: Send + Sync {
    /// Wait for the next update. Returns `None` when the monitor is closed.
    fn poll(
        &mut self,
    ) -> impl std::future::Future<Output = Option<PvStructure>> + Send;

    /// Start the monitor (begin receiving events).
    fn start(
        &mut self,
    ) -> impl std::future::Future<Output = BridgeResult<()>> + Send;

    /// Stop the monitor.
    fn stop(&mut self) -> impl std::future::Future<Output = ()> + Send;
}

// ---------------------------------------------------------------------------
// AnyChannel — enum dispatch for Channel trait
// ---------------------------------------------------------------------------

/// Concrete channel type returned by BridgeProvider.
///
/// Uses enum dispatch instead of `dyn Channel` because async trait methods
/// with `impl Future` return types are not dyn-compatible.
pub enum AnyChannel {
    Single(BridgeChannel),
    Group(GroupChannel),
}

impl Channel for AnyChannel {
    fn channel_name(&self) -> &str {
        match self {
            Self::Single(ch) => ch.channel_name(),
            Self::Group(ch) => ch.channel_name(),
        }
    }

    async fn get(&self, request: &PvStructure) -> BridgeResult<PvStructure> {
        match self {
            Self::Single(ch) => ch.get(request).await,
            Self::Group(ch) => ch.get(request).await,
        }
    }

    async fn put(&self, value: &PvStructure) -> BridgeResult<()> {
        match self {
            Self::Single(ch) => ch.put(value).await,
            Self::Group(ch) => ch.put(value).await,
        }
    }

    async fn get_field(&self) -> BridgeResult<FieldDesc> {
        match self {
            Self::Single(ch) => ch.get_field().await,
            Self::Group(ch) => ch.get_field().await,
        }
    }

    async fn create_monitor(&self) -> BridgeResult<crate::group::AnyMonitor> {
        match self {
            Self::Single(ch) => ch.create_monitor().await,
            Self::Group(ch) => ch.create_monitor().await,
        }
    }
}

// ---------------------------------------------------------------------------
// BridgeProvider
// ---------------------------------------------------------------------------

/// Bridge ChannelProvider that exposes EPICS database records as PVA channels.
///
/// Corresponds to C++ `PDBProvider`.
pub struct BridgeProvider {
    db: Arc<PvDatabase>,
    groups: HashMap<String, GroupPvDef>,
}

impl BridgeProvider {
    pub fn new(db: Arc<PvDatabase>) -> Self {
        Self {
            db,
            groups: HashMap::new(),
        }
    }

    /// Load group PV definitions from a JSON config string.
    pub fn load_group_config(&mut self, json: &str) -> BridgeResult<()> {
        let defs = crate::group_config::parse_group_config(json)?;
        for def in defs {
            self.groups.insert(def.name.clone(), def);
        }
        Ok(())
    }

    /// Load group PV definitions from a JSON file.
    pub fn load_group_file(&mut self, path: &str) -> BridgeResult<()> {
        let content = std::fs::read_to_string(path)?;
        self.load_group_config(&content)
    }

    /// Access the underlying database.
    pub fn database(&self) -> &Arc<PvDatabase> {
        &self.db
    }

    /// Access group definitions.
    pub fn groups(&self) -> &HashMap<String, GroupPvDef> {
        &self.groups
    }
}

impl ChannelProvider for BridgeProvider {
    fn provider_name(&self) -> &str {
        "BRIDGE"
    }

    async fn channel_find(&self, name: &str) -> bool {
        if self.groups.contains_key(name) {
            return true;
        }
        self.db.has_name(name).await
    }

    async fn channel_list(&self) -> Vec<String> {
        let mut names = self.db.all_record_names().await;
        names.extend(self.groups.keys().cloned());
        names.sort();
        names
    }

    async fn create_channel(&self, name: &str) -> BridgeResult<AnyChannel> {
        // Check group PVs first
        if let Some(def) = self.groups.get(name) {
            return Ok(AnyChannel::Group(GroupChannel::new(
                self.db.clone(),
                def.clone(),
            )));
        }

        // Single record channel
        if self.db.has_name(name).await {
            let channel = BridgeChannel::new(self.db.clone(), name).await?;
            return Ok(AnyChannel::Single(channel));
        }

        Err(BridgeError::ChannelNotFound(name.to_string()))
    }
}
