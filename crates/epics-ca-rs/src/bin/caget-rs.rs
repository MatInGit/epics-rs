use clap::Parser;
use epics_ca_rs::client::CaClient;
use std::time::Duration;

#[derive(Parser)]
#[command(name = "caget", about = "Read EPICS PV values")]
struct Args {
    /// CA timeout in seconds (default: 1.0)
    #[arg(short = 'w', long = "wait", default_value_t = 1.0)]
    timeout: f64,

    /// PV names to read
    #[arg(required = true)]
    pv_names: Vec<String>,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let client = CaClient::new().await.expect("failed to create CA client");
    let timeout = Duration::from_secs_f64(args.timeout);

    let mut failed = false;
    for pv_name in &args.pv_names {
        let ch = client.create_channel(pv_name);
        match ch.wait_connected(timeout).await {
            Ok(()) => match ch.get().await {
                Ok((_dbr_type, value)) => {
                    println!("{pv_name} {value}");
                }
                Err(e) => {
                    eprintln!("{pv_name}: {e}");
                    failed = true;
                }
            },
            Err(_) => {
                eprintln!("{pv_name}: *** Not connected (PV not found)");
                failed = true;
            }
        }
    }
    if failed {
        std::process::exit(1);
    }
}
