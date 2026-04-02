# epics-rs

Umbrella crate that re-exports all epics-rs sub-crates. Use feature flags to select which modules you need.

## Features

| Feature | Description | Default |
|---------|-------------|---------|
| `ca` | Channel Access client & server | yes |
| `pva` | pvAccess client (experimental) | no |
| `asyn` | Async port driver framework | no |
| `motor` | Motor record + SimMotor | no |
| `ad` | areaDetector (core + plugins) | no |
| `calc` | Calc expression engine | always |
| `autosave` | PV save/restore | always |
| `busy` | Busy record | always |
| `std` | Standard records (epid, throttle, timestamp) | no |
| `scaler` | Scaler record (multi-channel counter) | no |
| `optics` | Beamline optics (table, monochromator, filters) | no |
| `full` | Everything | no |

## Usage

```toml
[dependencies]
epics-rs = { version = "0.7", features = ["motor", "ad"] }
```

## License

[EPICS Open License](../../LICENSE)
