# Changelog

## v0.7.0

- **Breaking**: Separate Channel Access into `epics-ca-rs` crate
- **Breaking**: Separate pvAccess into `epics-pva-rs` crate
- **Breaking**: Rename crates for consistent `-rs` suffix (ad-core-rs, ad-plugins-rs, epics-macros-rs, epics-seq-rs, snc-core-rs, snc-rs)
- Add `epics-rs` umbrella crate with feature flags (ca, pva, motor, ad, calc, full, etc.)
- Remove msi from workspace (moved to separate repo)
- Add 113 C EPICS parity tests (ai/bi/bo record, deadband, alarm, calc engine, FLNK chains, CA wire protocol, .db parsing, autosave)
- Add SAFETY comments for production unwrap sites
- Clippy lint cleanup across all crates

## v0.6.1

- Fix monitor deadband for records without MDEL field
- Reset beacon interval on TCP connect/disconnect (C EPICS parity)
- Fix caput-rs to use fire-and-forget write like C caput, add `-c` flag for callback mode
- Show Old/New values in caput-rs output
- Support multiple PV names in CA/PVA CLI tools (caget, camonitor, cainfo, pvaget, etc.)
- Add per-field change detection for monitor notifications
- Add DMOV same-position transition tests
- Poll motor immediately on StartPolling for faster DMOV response
- Add motor tests ported from ophyd (sequential moves, calibration, RBV updates, homing)
- Update minimum Rust version to 1.85+ for edition 2024

## v0.6.0

- Deferred write_notify via callback for motor records
- Motor display/ctrl metadata support
- SET mode RBV updates

## v0.5.2

- Fix monitor notify, DMOV transition, timestamp, and IPv4 resolution

## v0.5.1

- Add DMOV 1->0->1 monitor transition for motor moves

## v0.5.0

- Fix motor record process chain, client error handling, and connection speed
- Add ophyd-test-ioc example

## v0.4.6

- Add client-side DBR_TIME/CTRL decode and get_with_metadata() API

## v0.4.5

- Upgrade Rust edition 2021 -> 2024

## v0.4.4

- Bug fixes

## v0.4.3

- Add generalTime framework for priority-based time providers
- Add random-signals example
- Add GitHub Actions CI workflow

## v0.4.2

- Implement C-compatible autosave iocsh commands and request file infrastructure

## v0.4.1

- Implement full YUV color mode support and refactor color convert plugin

## v0.4.0

- Initial crates.io publish
- Move to epics-rs GitHub organization

## v0.3.0

- Unify workspace version management
