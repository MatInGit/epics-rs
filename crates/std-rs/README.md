# std-rs

Rust port of the EPICS [std module](https://github.com/epics-modules/std) — standard records and device support for IOC applications.

## Record Types

- **epid** — Enhanced PID controller record
- **throttle** — Rate-limiting output record
- **timestamp** — Timestamp capture record

## Usage

```toml
[dependencies]
std-rs = { workspace = true }
```

Register all record types in your IOC:

```rust
use std_rs::std_record_factories;

for (name, factory) in std_record_factories() {
    app.register_record_type(name, factory);
}
```

## License

[EPICS Open License](../../LICENSE)
