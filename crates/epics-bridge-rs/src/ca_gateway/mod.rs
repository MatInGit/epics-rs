//! # ca_gateway — CA fan-out gateway (C++ ca-gateway equivalent)
//!
//! Pure Rust port of [EPICS ca-gateway](https://github.com/epics-modules/ca-gateway).
//! A Channel Access proxy that:
//!
//! - Accepts downstream client connections (CA server side)
//! - Connects to upstream IOCs (CA client side)
//! - Caches PV values and fans out monitor events to multiple clients
//! - Applies access security rules from a `.pvlist` file
//! - Supports PV name aliasing with regex backreferences
//! - Tracks per-PV statistics and exposes them as PVs
//!
//! ## Architecture
//!
//! ```text
//! Upstream IOCs                Gateway                 Downstream Clients
//! ┌─────────┐                ┌─────────┐               ┌─────────┐
//! │ IOC #1  │ ◄── CaClient ──┤         ├── CaServer ──►│ caget   │
//! └─────────┘                │ PvCache │               └─────────┘
//! ┌─────────┐                │  + ACL  │               ┌─────────┐
//! │ IOC #2  │ ◄── CaClient ──┤  + Stats├── CaServer ──►│  CSS    │
//! └─────────┘                │         │               └─────────┘
//!                            └─────────┘                  (~1000)
//! ```
//!
//! ## Sub-modules
//!
//! - [`cache`] — PvCache, GwPvEntry, PvState (5-state FSM)
//! - [`pvlist`] — `.pvlist` configuration file parser
//! - `access` — access security adapter (planned)
//! - `upstream` — CaClient adapter (planned)
//! - `downstream` — CaServer adapter (planned)
//! - `stats` — gateway statistics PVs (planned)
//! - `server` — GatewayServer top-level (planned)
//!
//! ## Status
//!
//! Skeleton phase. Core data structures and config parser are implemented
//! and tested. Upstream/downstream adapters and the main event loop will
//! be added in subsequent PRs.

pub mod cache;
pub mod pvlist;

pub use cache::{GwPvEntry, PvCache, PvState};
pub use pvlist::{PvList, PvListEntry, PvListMatch, EvaluationOrder};
