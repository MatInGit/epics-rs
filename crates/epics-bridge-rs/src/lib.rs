//! # epics-bridge-rs
//!
//! Bridge that exposes EPICS database records as pvAccess channels
//! (NTScalar, NTEnum, NTScalarArray, Group PV).
//!
//! This crate corresponds to C++ EPICS QSRV (`modules/pva2pva/pdbApp/`).
//! It translates between `epics-base-rs` record state and `epics-pva-rs`
//! PVA data structures.
//!
//! ## Architecture
//!
//! ```text
//! PVA Client ←→ [epics-pva-rs server] ←→ BridgeProvider ←→ PvDatabase
//! ```
//!
//! - [`BridgeProvider`] implements [`ChannelProvider`] — the PVA server calls
//!   into it to resolve channel names and create channels.
//! - [`BridgeChannel`] serves single-record PVs (NTScalar, NTEnum, NTScalarArray).
//! - [`GroupChannel`] serves multi-record composite PVs from JSON config.
//! - [`BridgeMonitor`] bridges `DbSubscription` events to PVA monitor updates.
//!
//! ## Note
//!
//! The `ChannelProvider`, `Channel`, and `PvaMonitor` traits are defined here
//! temporarily. They will move to `epics-pva-rs` once the PVA server is
//! implemented by the spvirit maintainer.

pub mod channel;
pub mod convert;
pub mod error;
pub mod group;
pub mod group_config;
pub mod monitor;
pub mod provider;
pub mod pvif;

pub use channel::{BridgeChannel, ProcessMode, PutOptions};
pub use error::{BridgeError, BridgeResult};
pub use group::{AnyMonitor, GroupChannel, GroupMonitor};
pub use group_config::GroupPvDef;
pub use monitor::BridgeMonitor;
pub use provider::{
    AccessControl, AllowAllAccess, AnyChannel, BridgeProvider, Channel, ChannelProvider,
    PvaMonitor,
};
pub use pvif::{FieldMapping, NtType};
