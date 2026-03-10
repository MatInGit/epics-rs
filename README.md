# epics-rs

Pure Rust implementation of the [EPICS](https://epics-controls.org/) control system framework.

## Workspace Structure

```
epics-rs/
├── crates/
│   ├── epics-base/      # Channel Access client/server, IOC runtime, PV Access
│   ├── epics-macros/     # Proc-macro helpers for epics-base
│   ├── msi/              # Macro Substitution and Include tool (.template → .db)
│   ├── asyn/             # Async device I/O framework (port driver model)
│   ├── autosave/         # Automatic PV save/restore
│   ├── calc/             # Calc expression engine (numeric, string, array)
│   ├── busy/             # Busy record support
│   ├── seq/              # Sequencer runtime (state machine engine)
│   ├── snc-core/         # SNL compiler library
│   ├── snc/              # SNL compiler CLI
│   ├── motor/            # Motor record and axis control
│   ├── ad-core/          # areaDetector core (NDArray, driver base)
│   ├── ad-plugins/       # areaDetector plugins (ROI, Stats, File, etc.)
│   └── sim-detector/     # Simulated areaDetector driver
└── examples/
    └── seq-demo/         # Sequencer demo application
```

## Build

```bash
cargo build --workspace
```

## Test

```bash
# Run all tests (1290+)
cargo test --workspace

# With optional feature flags
cargo test --workspace --features calc-rs/epics,asyn-rs/epics
```

## Binaries

```bash
# Channel Access tools
cargo build --bin caget-rs
cargo build --bin caput-rs
cargo build --bin camonitor-rs
cargo build --bin cainfo-rs
cargo build --bin ca-repeater-rs

# PV Access tools
cargo build --bin pvaget-rs
cargo build --bin pvaput-rs
cargo build --bin pvamonitor-rs
cargo build --bin pvainfo-rs

# Soft IOC
cargo build --bin softioc-rs

# SNL compiler
cargo build --bin snc

# MSI tool
cargo build --bin msi-rs --features msi-rs/cli

# Simulated detector IOC
cargo build --bin sim_ioc --features sim-detector/ioc
```

## Feature Flags

| Crate | Feature | Description |
|-------|---------|-------------|
| `asyn` | `epics` | Enables adapter bridging to epics-base |
| `calc` | `epics` | Enables string, array, and epics-base integration |
| `calc` | `numeric` | Numeric expressions (default) |
| `calc` | `string` | String expressions |
| `calc` | `array` | Array expressions |
| `msi` | `cli` | Enables the `msi-rs` CLI binary |
| `sim-detector` | `ioc` | Enables `sim_ioc` binary with full IOC support |

## License

This project is for research and development purposes.
