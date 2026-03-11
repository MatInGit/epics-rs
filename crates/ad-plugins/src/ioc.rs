//! IOC plugin registration: C-compatible configure commands for all AD plugins.
//!
//! Provides [`register_all_plugins`] which registers startup commands like
//! `NDStatsConfigure`, `NDROIConfigure`, etc. on an `IocApplication`.

use std::sync::Arc;

use ad_core::ioc::{dtyp_from_port, extract_plugin_args, plugin_arg_defs, PluginManager};
use ad_core::plugin::runtime::create_plugin_runtime;
use epics_base_rs::server::ioc_app::IocApplication;
use epics_base_rs::server::iocsh::registry::*;

/// Register all standard areaDetector plugin configure commands.
///
/// The `PluginManager` must have its driver context set (via `set_driver()`)
/// before any of these commands are invoked from st.cmd.
pub fn register_all_plugins(
    mut app: IocApplication,
    mgr: &Arc<PluginManager>,
) -> IocApplication {
    // --- NDStdArraysConfigure ---
    {
        let m = mgr.clone();
        app = app.register_startup_command(CommandDef::new(
            "NDStdArraysConfigure",
            plugin_arg_defs(),
            "NDStdArraysConfigure portName [queueSize] ...",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let (port_name, _queue_size, ndarray_port) = extract_plugin_args(args)?;
                let dtyp = dtyp_from_port(&port_name);
                let drv = m.driver()?;
                let pool = drv.pool();
                let (handle, data, _jh) =
                    crate::std_arrays::create_std_arrays_runtime(&port_name, pool, &ndarray_port);
                drv.connect_downstream(handle.array_sender().clone());
                println!("NDStdArraysConfigure: port={port_name}");
                m.add_plugin(&dtyp, &handle, Some(data));
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // --- NDStatsConfigure ---
    {
        let m = mgr.clone();
        app = app.register_startup_command(CommandDef::new(
            "NDStatsConfigure",
            plugin_arg_defs(),
            "NDStatsConfigure portName [queueSize] ...",
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let (port_name, queue_size, ndarray_port) = extract_plugin_args(args)?;
                let dtyp = dtyp_from_port(&port_name);
                let drv = m.driver()?;
                let pool = drv.pool();
                let (handle, _stats, stats_params, ts_runtime, ts_params, _jh, _ts_actor_jh, _ts_data_jh) =
                    crate::stats::create_stats_runtime(&port_name, pool, queue_size, &ndarray_port);
                drv.connect_downstream(handle.array_sender().clone());
                println!("NDStatsConfigure: port={port_name}");

                let registry = Arc::new(crate::stats::build_stats_registry(&handle, &stats_params));
                m.add_plugin_with_registry(&dtyp, &handle, registry, None);

                // Register TimeSeries as a separate asyn port
                let ts_port_name = format!("{port_name}_TS");
                let ts_dtyp = dtyp_from_port(&ts_port_name);
                let ts_registry = Arc::new(crate::time_series::build_ts_registry(&ts_params));
                let ts_port_handle = ts_runtime.port_handle().clone();
                m.add_port(&ts_dtyp, ts_port_handle, ts_registry);
                println!("  TimeSeries port: {ts_port_name} (DTYP: {ts_dtyp})");

                Ok(CommandOutcome::Continue)
            },
        ));
    }

    // --- Generic plugins using create_plugin_runtime ---
    app = register_generic_plugin(&mut app, mgr, "NDROIConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::roi::{ROIConfig, ROIProcessor};
        create_plugin_runtime(port_name, ROIProcessor::new(ROIConfig::default()), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDProcessConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::process::{ProcessConfig, ProcessProcessor};
        create_plugin_runtime(port_name, ProcessProcessor::new(ProcessConfig::default()), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDTransformConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::transform::{TransformType, TransformProcessor};
        create_plugin_runtime(port_name, TransformProcessor::new(TransformType::None), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDColorConvertConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::color_convert::{ColorConvertConfig, ColorConvertProcessor};
        use ad_core::color::{NDColorMode, NDBayerPattern};
        let config = ColorConvertConfig { target_mode: NDColorMode::Mono, bayer_pattern: NDBayerPattern::RGGB, false_color: false };
        create_plugin_runtime(port_name, ColorConvertProcessor::new(config), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDOverlayConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::overlay::OverlayProcessor;
        create_plugin_runtime(port_name, OverlayProcessor::new(vec![]), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDFFTConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::fft::{FFTMode, FFTProcessor};
        create_plugin_runtime(port_name, FFTProcessor::new(FFTMode::Rows1D), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDCircularBuffConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::circular_buff::{CircularBuffProcessor, TriggerCondition};
        create_plugin_runtime(port_name, CircularBuffProcessor::new(100, 100, TriggerCondition::External), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDCodecConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::codec::{CodecMode, CodecProcessor};
        use ad_core::codec::CodecName;
        create_plugin_runtime(port_name, CodecProcessor::new(CodecMode::Compress { codec: CodecName::LZ4, quality: 90 }), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDScatterConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::scatter::ScatterProcessor;
        create_plugin_runtime(port_name, ScatterProcessor::new(), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDGatherConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::gather::GatherProcessor;
        create_plugin_runtime(port_name, GatherProcessor::new(), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDFileTIFFConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::file_tiff::TiffFileProcessor;
        create_plugin_runtime(port_name, TiffFileProcessor::new(), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDFileJPEGConfigure", |port_name, queue_size, ndarray_port, pool| {
        use crate::file_jpeg::JpegFileProcessor;
        create_plugin_runtime(port_name, JpegFileProcessor::new(90), pool, queue_size, ndarray_port)
    });
    app = register_generic_plugin(&mut app, mgr, "NDFileHDF5Configure", |port_name, queue_size, ndarray_port, pool| {
        use crate::file_hdf5::Hdf5FileProcessor;
        create_plugin_runtime(port_name, Hdf5FileProcessor::new(), pool, queue_size, ndarray_port)
    });

    // --- Stub plugins (not yet fully implemented, use PassthroughProcessor) ---
    for name in &[
        "NDROIStatConfigure",
        "NDAttrConfigure",
        "NDBadPixelConfigure",
        "NDFileNetCDFConfigure",
        "NDFileNexusConfigure",
        "NDFileMagickConfigure",
        "NDTimeSeriesConfigure",
        "NDPvaConfigure",
    ] {
        let cmd_name = *name;
        let m = mgr.clone();
        app = app.register_startup_command(CommandDef::new(
            cmd_name,
            plugin_arg_defs(),
            &format!("{cmd_name} portName [queueSize] ... (stub)"),
            move |args: &[ArgValue], _ctx: &CommandContext| {
                let (port_name, queue_size, ndarray_port) = extract_plugin_args(args)?;
                let dtyp = dtyp_from_port(&port_name);
                let drv = m.driver()?;
                let pool = drv.pool();
                use crate::passthrough::PassthroughProcessor;
                let (handle, _jh) = create_plugin_runtime(
                    &port_name,
                    PassthroughProcessor::new(cmd_name),
                    pool,
                    queue_size,
                    &ndarray_port,
                );
                drv.connect_downstream(handle.array_sender().clone());
                println!("{cmd_name}: port={port_name} (stub)");
                m.add_plugin(&dtyp, &handle, None);
                Ok(CommandOutcome::Continue)
            },
        ));
    }

    app
}

/// Helper: register a generic plugin configure command that follows the standard pattern.
fn register_generic_plugin<F>(
    app: &mut IocApplication,
    mgr: &Arc<PluginManager>,
    cmd_name: &'static str,
    factory: F,
) -> IocApplication
where
    F: Fn(
            &str,
            usize,
            &str,
            Arc<ad_core::ndarray_pool::NDArrayPool>,
        ) -> (
            ad_core::plugin::runtime::PluginRuntimeHandle,
            std::thread::JoinHandle<()>,
        ) + Send
        + Sync
        + 'static,
{
    let m = mgr.clone();
    // Take ownership of app temporarily via a dummy
    let taken = std::mem::replace(app, IocApplication::new());
    let result = taken.register_startup_command(CommandDef::new(
        cmd_name,
        plugin_arg_defs(),
        &format!("{cmd_name} portName [queueSize] ..."),
        move |args: &[ArgValue], _ctx: &CommandContext| {
            let (port_name, queue_size, ndarray_port) = extract_plugin_args(args)?;
            let dtyp = dtyp_from_port(&port_name);
            let drv = m.driver()?;
            let pool = drv.pool();
            let (handle, _jh) = factory(&port_name, queue_size, &ndarray_port, pool);
            drv.connect_downstream(handle.array_sender().clone());
            println!("{cmd_name}: port={port_name}");
            m.add_plugin(&dtyp, &handle, None);
            Ok(CommandOutcome::Continue)
        },
    ));
    result
}
