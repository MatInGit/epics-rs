# scaler-rs

Rust port of the [EPICS scaler record](https://github.com/epics-modules/scaler) — multi-channel counter/timer with preset and auto-count support.

## Features

- ScalerRecord with configurable channel count
- Device support via asyn port driver interface
- Bundled database templates (`db/`)

## Usage

```toml
[dependencies]
scaler-rs = { workspace = true }
```

Register the record type in your IOC:

```rust
use scaler_rs::scaler_record_factory;

let (name, factory) = scaler_record_factory();
app.register_record_type(name, factory);
```

## License

[EPICS Open License](../../LICENSE)
