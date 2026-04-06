use std::collections::HashMap;
use std::net::SocketAddr;

use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpStream;
use epics_base_rs::runtime::sync::mpsc;

use crate::channel::AccessRights;
use crate::protocol::*;

use super::types::{TransportCommand, TransportEvent};

struct ServerConnection {
    write_tx: mpsc::UnboundedSender<Vec<u8>>,
    _read_task: tokio::task::JoinHandle<()>,
    _write_task: tokio::task::JoinHandle<()>,
}

pub(crate) async fn run_transport_manager(
    mut command_rx: mpsc::UnboundedReceiver<TransportCommand>,
    event_tx: mpsc::UnboundedSender<TransportEvent>,
) {
    let mut connections: HashMap<SocketAddr, ServerConnection> = HashMap::new();

    while let Some(cmd) = command_rx.recv().await {
        match cmd {
            TransportCommand::CreateChannel {
                cid,
                pv_name,
                server_addr,
            } => {
                // Ensure we have a connection to this server
                if !connections.contains_key(&server_addr) {
                    match connect_server(server_addr, event_tx.clone()).await {
                        Some(conn) => {
                            connections.insert(server_addr, conn);
                        }
                        None => {
                            let _ = event_tx.send(TransportEvent::TcpClosed { server_addr });
                            continue;
                        }
                    }
                }

                // Check connection is still alive (both tasks running)
                let alive = connections
                    .get(&server_addr)
                    .map(|c| !c._read_task.is_finished() && !c._write_task.is_finished())
                    .unwrap_or(false);

                if !alive {
                    connections.remove(&server_addr);
                    match connect_server(server_addr, event_tx.clone()).await {
                        Some(conn) => {
                            connections.insert(server_addr, conn);
                        }
                        None => {
                            let _ = event_tx.send(TransportEvent::TcpClosed { server_addr });
                            continue;
                        }
                    }
                }

                let pv_payload = pad_string(&pv_name);
                let mut create_hdr = CaHeader::new(CA_PROTO_CREATE_CHAN);
                create_hdr.postsize = pv_payload.len() as u16;
                create_hdr.cid = cid;
                create_hdr.available = CA_MINOR_VERSION as u32;

                let mut frame = create_hdr.to_bytes().to_vec();
                frame.extend_from_slice(&pv_payload);
                send_frame(&mut connections, server_addr, frame, &event_tx);
            }
            TransportCommand::ReadNotify {
                sid,
                data_type,
                count,
                ioid,
                server_addr,
            } => {
                let mut hdr = CaHeader::new(CA_PROTO_READ_NOTIFY);
                hdr.data_type = data_type;
                hdr.cid = sid;
                hdr.available = ioid;
                if count > 0xFFFF {
                    hdr.set_payload_size(0, count);
                } else {
                    hdr.count = count as u16;
                }
                send_frame(&mut connections, server_addr, hdr.to_bytes_extended(), &event_tx);
            }
            TransportCommand::Write {
                sid,
                data_type,
                count,
                payload,
                server_addr,
            } => {
                let padded_len = align8(payload.len());
                let mut padded = payload;
                padded.resize(padded_len, 0);

                let mut hdr = CaHeader::new(CA_PROTO_WRITE);
                hdr.data_type = data_type;
                hdr.cid = sid;
                hdr.set_payload_size(padded.len(), count);

                let mut frame = hdr.to_bytes_extended();
                frame.extend_from_slice(&padded);
                send_frame(&mut connections, server_addr, frame, &event_tx);
            }
            TransportCommand::WriteNotify {
                sid,
                data_type,
                count,
                ioid,
                payload,
                server_addr,
            } => {
                let padded_len = align8(payload.len());
                let mut padded = payload;
                padded.resize(padded_len, 0);

                let mut hdr = CaHeader::new(CA_PROTO_WRITE_NOTIFY);
                hdr.data_type = data_type;
                hdr.cid = sid;
                hdr.available = ioid;
                hdr.set_payload_size(padded.len(), count);

                let mut frame = hdr.to_bytes_extended();
                frame.extend_from_slice(&padded);
                send_frame(&mut connections, server_addr, frame, &event_tx);
            }
            TransportCommand::Subscribe {
                sid,
                data_type,
                count,
                subid,
                mask,
                server_addr,
            } => {
                let mut hdr = CaHeader::new(CA_PROTO_EVENT_ADD);
                hdr.postsize = 16;
                hdr.data_type = data_type;
                hdr.cid = sid;
                hdr.available = subid;
                if count > 0xFFFF {
                    hdr.set_payload_size(16, count);
                } else {
                    hdr.count = count as u16;
                }

                let mut mask_payload = [0u8; 16];
                mask_payload[12..14].copy_from_slice(&mask.to_be_bytes());

                let mut frame = hdr.to_bytes_extended();
                frame.extend_from_slice(&mask_payload);
                send_frame(&mut connections, server_addr, frame, &event_tx);
            }
            TransportCommand::Unsubscribe {
                sid,
                subid,
                data_type,
                server_addr,
            } => {
                let mut hdr = CaHeader::new(CA_PROTO_EVENT_CANCEL);
                hdr.data_type = data_type;
                hdr.cid = sid;
                hdr.available = subid;
                send_frame(&mut connections, server_addr, hdr.to_bytes().to_vec(), &event_tx);
            }
            TransportCommand::ClearChannel {
                cid,
                sid,
                server_addr,
            } => {
                let mut hdr = CaHeader::new(CA_PROTO_CLEAR_CHANNEL);
                hdr.cid = sid;
                hdr.available = cid;
                send_frame(&mut connections, server_addr, hdr.to_bytes().to_vec(), &event_tx);
            }
        }
    }
}

fn send_frame(
    connections: &mut HashMap<SocketAddr, ServerConnection>,
    server_addr: SocketAddr,
    frame: Vec<u8>,
    event_tx: &mpsc::UnboundedSender<TransportEvent>,
) {
    let failed = connections
        .get(&server_addr)
        .is_some_and(|c| c.write_tx.send(frame).is_err());
    if failed {
        connections.remove(&server_addr);
        let _ = event_tx.send(TransportEvent::TcpClosed { server_addr });
    }
}

async fn connect_server(
    server_addr: SocketAddr,
    event_tx: mpsc::UnboundedSender<TransportEvent>,
) -> Option<ServerConnection> {
    let stream = tokio::time::timeout(
        std::time::Duration::from_secs(5),
        TcpStream::connect(server_addr),
    )
    .await
    .ok()?
    .ok()?;

    let _ = stream.set_nodelay(true);

    let (reader, write_half) = stream.into_split();
    let (write_tx, write_rx) = mpsc::unbounded_channel();

    // Build initial handshake as a single frame (VERSION + HOST + CLIENT)
    let mut handshake = Vec::new();

    let mut version_hdr = CaHeader::new(CA_PROTO_VERSION);
    version_hdr.count = CA_MINOR_VERSION;
    handshake.extend_from_slice(&version_hdr.to_bytes());

    let hostname = epics_base_rs::runtime::env::hostname();
    let host_payload = pad_string(&hostname);
    let mut host_hdr = CaHeader::new(CA_PROTO_HOST_NAME);
    host_hdr.postsize = host_payload.len() as u16;
    handshake.extend_from_slice(&host_hdr.to_bytes());
    handshake.extend_from_slice(&host_payload);

    let username = epics_base_rs::runtime::env::get("USER")
        .or_else(|| epics_base_rs::runtime::env::get("USERNAME"))
        .unwrap_or_else(|| "unknown".to_string());
    let user_payload = pad_string(&username);
    let mut user_hdr = CaHeader::new(CA_PROTO_CLIENT_NAME);
    user_hdr.postsize = user_payload.len() as u16;
    handshake.extend_from_slice(&user_hdr.to_bytes());
    handshake.extend_from_slice(&user_payload);

    let _ = write_tx.send(handshake);

    let write_task = epics_base_rs::runtime::task::spawn(write_loop(write_half, write_rx, server_addr, event_tx.clone()));
    let read_task = epics_base_rs::runtime::task::spawn(read_loop(reader, server_addr, event_tx, write_tx.clone()));

    Some(ServerConnection {
        write_tx,
        _read_task: read_task,
        _write_task: write_task,
    })
}

async fn write_loop(
    mut writer: tokio::net::tcp::OwnedWriteHalf,
    mut rx: mpsc::UnboundedReceiver<Vec<u8>>,
    server_addr: SocketAddr,
    event_tx: mpsc::UnboundedSender<TransportEvent>,
) {
    let mut batch = Vec::with_capacity(4096);
    while let Some(frame) = rx.recv().await {
        batch.extend_from_slice(&frame);
        // Drain all pending frames into a single write
        while let Ok(frame) = rx.try_recv() {
            batch.extend_from_slice(&frame);
        }
        if writer.write_all(&batch).await.is_err() {
            let _ = event_tx.send(TransportEvent::TcpClosed { server_addr });
            return;
        }
        batch.clear();
    }
}

async fn read_loop(
    mut reader: tokio::net::tcp::OwnedReadHalf,
    server_addr: SocketAddr,
    event_tx: mpsc::UnboundedSender<TransportEvent>,
    write_tx: mpsc::UnboundedSender<Vec<u8>>,
) {
    let mut buf = vec![0u8; 8192];
    let mut accumulated = Vec::new();

    loop {
        let n = match reader.read(&mut buf).await {
            Ok(0) | Err(_) => {
                let _ = event_tx.send(TransportEvent::TcpClosed { server_addr });
                return;
            }
            Ok(n) => n,
        };

        accumulated.extend_from_slice(&buf[..n]);

        let mut offset = 0;
        while offset + CaHeader::SIZE <= accumulated.len() {
            let (hdr, hdr_size) = match CaHeader::from_bytes_extended(&accumulated[offset..]) {
                Ok(v) => v,
                Err(_) => break,
            };
            let actual_post = hdr.actual_postsize();
            let msg_len = hdr_size + align8(actual_post);

            if offset + msg_len > accumulated.len() {
                break;
            }

            let data_start = offset + hdr_size;

            match hdr.cmmd {
                CA_PROTO_VERSION => {}
                CA_PROTO_ACCESS_RIGHTS => {
                    let _ = event_tx.send(TransportEvent::AccessRightsChanged {
                        cid: hdr.cid,
                        access: AccessRights::from_u32(hdr.available),
                    });
                }
                CA_PROTO_CREATE_CHAN => {
                    let _ = event_tx.send(TransportEvent::ChannelCreated {
                        cid: hdr.cid,
                        sid: hdr.available,
                        data_type: hdr.data_type,
                        element_count: hdr.actual_count(),
                        access: AccessRights::from_u32(0x3), // default; real access comes via ACCESS_RIGHTS
                        server_addr,
                    });
                }
                CA_PROTO_READ_NOTIFY => {
                    if hdr.cid == ECA_NORMAL {
                        let data = accumulated[data_start..data_start + actual_post].to_vec();
                        let _ = event_tx.send(TransportEvent::ReadResponse {
                            ioid: hdr.available,
                            data_type: hdr.data_type,
                            count: hdr.actual_count(),
                            data,
                        });
                    } else {
                        let _ = event_tx.send(TransportEvent::ReadError {
                            ioid: hdr.available,
                            eca_status: hdr.cid,
                        });
                    }
                }
                CA_PROTO_WRITE_NOTIFY => {
                    // Per spec: param1 (cid field) = ECA status, param2 (available field) = IOID
                    let _ = event_tx.send(TransportEvent::WriteResponse {
                        ioid: hdr.available,
                        status: hdr.cid,
                    });
                }
                CA_PROTO_EVENT_ADD => {
                    let data = accumulated[data_start..data_start + actual_post].to_vec();
                    let _ = event_tx.send(TransportEvent::MonitorData {
                        subid: hdr.available,
                        data_type: hdr.data_type,
                        count: hdr.actual_count(),
                        data,
                    });
                }
                CA_PROTO_ECHO => {
                    let echo_hdr = CaHeader::new(CA_PROTO_ECHO);
                    let _ = write_tx.send(echo_hdr.to_bytes().to_vec());
                }
                CA_PROTO_CREATE_CH_FAIL => {
                    let _ = event_tx.send(TransportEvent::ChannelCreateFailed {
                        cid: hdr.cid,
                    });
                }
                CA_PROTO_ERROR => {
                    // Payload contains original request header (16 bytes) + error message string
                    let orig_cmd = if actual_post >= 16 {
                        let orig_hdr_bytes = &accumulated[data_start..data_start + 16];
                        Some(u16::from_be_bytes([orig_hdr_bytes[0], orig_hdr_bytes[1]]))
                    } else {
                        None
                    };
                    let msg = if actual_post > 16 {
                        let msg_bytes = &accumulated[data_start + 16..data_start + actual_post];
                        let end = msg_bytes.iter().position(|&b| b == 0).unwrap_or(msg_bytes.len());
                        String::from_utf8_lossy(&msg_bytes[..end]).to_string()
                    } else {
                        String::new()
                    };
                    eprintln!("CA server error: cmd={:?} msg={}", orig_cmd, msg);
                    let _ = event_tx.send(TransportEvent::ServerError {
                        _original_request: orig_cmd,
                        _message: msg,
                    });
                }
                CA_PROTO_SERVER_DISCONN => {
                    let _ = event_tx.send(TransportEvent::ServerDisconnect {
                        cid: hdr.cid,
                        server_addr,
                    });
                }
                _ => {}
            }

            offset += msg_len;
        }

        if offset > 0 {
            accumulated.drain(..offset);
        }
    }
}
