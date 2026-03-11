//! SimDetector IOC binary.
//!
//! Uses IocApplication for st.cmd-style startup matching the C++ EPICS pattern.
//! Plugin commands are provided by `ad_plugins::ioc::register_all_plugins()`.
//!
//! Usage:
//!   cargo run --bin sim_ioc --features ioc -- st.cmd
//!   cargo run --bin sim_ioc --features ioc -- ioc/st.cmd

use std::sync::Arc;

use asyn_rs::trace::TraceManager;
use epics_base_rs::error::CaResult;
use epics_base_rs::server::ioc_app::IocApplication;
use epics_base_rs::server::iocsh::registry::*;

use ad_core::ioc::{PluginManager, register_noop_commands};
use ad_core::plugin::channel::NDArrayOutput;
use sim_detector::driver::{create_sim_detector, SimDetectorRuntime};
use sim_detector::ioc_support::{build_param_registry_from_params, SimDeviceSupport};

#[tokio::main]
async fn main() -> CaResult<()> {
    let args: Vec<String> = std::env::args().collect();

    // Set C source tree paths for template include resolution
    epics_base_rs::runtime::env::set_default("ADCORE", concat!(env!("CARGO_MANIFEST_DIR"), "/../../crates/ad-core"));
    epics_base_rs::runtime::env::set_default("ADSIMDETECTOR", env!("CARGO_MANIFEST_DIR"));

    let script = if args.len() > 1 && !args[1].starts_with('-') {
        args[1].clone()
    } else {
        eprintln!("Usage: sim_ioc <st.cmd>");
        eprintln!();
        eprintln!("The st.cmd script should contain:");
        eprintln!(r#"  epicsEnvSet("PREFIX", "SIM1:")"#);
        eprintln!(r#"  simDetectorConfig("SIM1", 256, 256, 50000000)"#);
        eprintln!(r#"  dbLoadRecords("$(ADSIMDETECTOR)/db/simDetector.template", "P=$(PREFIX),R=cam1:,PORT=SIM1,DTYP=asynSimDetector")"#);
        std::process::exit(1);
    };

    // Register the full asynRecord type (overrides the minimal stub)
    asyn_rs::asyn_record::register_asyn_record_type();

    let trace = Arc::new(TraceManager::new());
    let mgr = PluginManager::new(trace.clone());

    // Shared state for the driver's device support factory
    let driver_port_handle: Arc<std::sync::Mutex<Option<asyn_rs::port_handle::PortHandle>>> =
        Arc::new(std::sync::Mutex::new(None));
    let driver_registry: Arc<std::sync::Mutex<Option<Arc<ad_core::plugin::registry::ParamRegistry>>>> =
        Arc::new(std::sync::Mutex::new(None));
    let driver_runtime: Arc<std::sync::Mutex<Option<SimDetectorRuntime>>> =
        Arc::new(std::sync::Mutex::new(None));

    let mut app = IocApplication::new();
    app = app.port(
        std::env::var("EPICS_CA_SERVER_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(5064),
    );

    // --- simDetectorConfig command ---
    {
        let mgr_c = mgr.clone();
        let ph = driver_port_handle.clone();
        let reg = driver_registry.clone();
        let rt = driver_runtime.clone();
        let trace_c = trace.clone();
        app = app.register_startup_command(CommandDef::new(
            "simDetectorConfig",
            vec![
                ArgDesc { name: "portName", arg_type: ArgType::String, optional: false },
                ArgDesc { name: "sizeX", arg_type: ArgType::Int, optional: true },
                ArgDesc { name: "sizeY", arg_type: ArgType::Int, optional: true },
                ArgDesc { name: "maxMemory", arg_type: ArgType::Int, optional: true },
            ],
            "simDetectorConfig portName [sizeX] [sizeY] [maxMemory]",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let port_name = match &args[0] {
                    ArgValue::String(s) => s.clone(),
                    _ => return Err("portName required".into()),
                };
                let size_x = match &args[1] {
                    ArgValue::Int(n) => *n as i32,
                    _ => 256,
                };
                let size_y = match &args[2] {
                    ArgValue::Int(n) => *n as i32,
                    _ => 256,
                };
                let max_memory = match &args[3] {
                    ArgValue::Int(n) => *n as usize,
                    _ => 50_000_000,
                };

                println!("simDetectorConfig: port={port_name}, size={size_x}x{size_y}, maxMemory={max_memory}");

                let array_output = NDArrayOutput::new();
                let runtime = create_sim_detector(&port_name, size_x, size_y, max_memory, array_output)
                    .map_err(|e| format!("failed to create SimDetector: {e}"))?;

                let registry = Arc::new(build_param_registry_from_params(&runtime.ad_params, &runtime.sim_params));
                let port_handle = runtime.port_handle().clone();

                asyn_rs::asyn_record::register_port(&port_name, port_handle.clone(), trace_c.clone());

                // Set driver context for plugin commands
                mgr_c.set_driver(Arc::new(SimDriverContext {
                    pool: runtime.pool().clone(),
                    runtime_output: runtime.array_output().clone(),
                }));

                *ph.lock().unwrap() = Some(port_handle);
                *reg.lock().unwrap() = Some(registry);
                *rt.lock().unwrap() = Some(runtime);

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // Register all plugin configure commands (NDStdArraysConfigure, NDStatsConfigure, etc.)
    app = ad_plugins::ioc::register_all_plugins(app, &mgr);

    // Register no-op commands from commonPlugins.cmd
    app = register_noop_commands(app);

    // Device support: detector
    {
        let ph = driver_port_handle.clone();
        let reg = driver_registry.clone();
        app = app.register_device_support("asynSimDetector", move || {
            let handle = ph.lock().unwrap()
                .as_ref()
                .expect("simDetectorConfig must be called before iocInit")
                .clone();
            let registry = reg.lock().unwrap()
                .as_ref()
                .expect("simDetectorConfig must be called before iocInit")
                .clone();
            Box::new(SimDeviceSupport::from_handle(handle, registry))
        });
    }

    // Device support: plugins (dynamic lookup by DTYP)
    app = mgr.register_device_support(app);

    // Shell command: simDetectorReport
    {
        let mgr_r = mgr.clone();
        app = app.register_shell_command(CommandDef::new(
            "simDetectorReport",
            vec![ArgDesc { name: "level", arg_type: ArgType::Int, optional: true }],
            "simDetectorReport [level] - Report SimDetector status",
            move |_args: &[ArgValue], _ctx: &CommandContext| {
                println!("SimDetector Report");
                mgr_r.report();
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app.startup_script(&script)
        .run()
        .await
}

/// DriverContext implementation for SimDetector.
struct SimDriverContext {
    pool: Arc<ad_core::ndarray_pool::NDArrayPool>,
    runtime_output: Arc<parking_lot::Mutex<NDArrayOutput>>,
}

impl ad_core::ioc::DriverContext for SimDriverContext {
    fn pool(&self) -> Arc<ad_core::ndarray_pool::NDArrayPool> {
        self.pool.clone()
    }

    fn connect_downstream(&self, sender: ad_core::plugin::channel::NDArraySender) {
        self.runtime_output.lock().add(sender);
    }
}
