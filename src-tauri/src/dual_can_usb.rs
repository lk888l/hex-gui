//! Two gs_usb channels sharing one USB claim, reader and serialized sender.
//! The wire format follows can-transport 0.1.4 (MIT OR Apache-2.0).
//! The ordinary single-channel backend remains in can-transport.

use anyhow::{bail, Context, Result};
use async_trait::async_trait;
use can_transport::gs_usb::{GsTiming, GsUsbConfig, KNOWN_IDS};
use can_transport::{
    CanBitTiming, CanBus, CanCapabilities, CanFilter, CanFrame, CanId, CanIoError, CanLinkConfig,
    CanRx, FrameKind,
};
use nusb::transfer::{Buffer, Bulk, ControlIn, ControlOut, ControlType, In, Out, Recipient};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex as StdMutex};
use std::time::Duration;
use tokio::sync::{mpsc, oneshot};

const USB_TIMEOUT: Duration = Duration::from_millis(750);
const DLC_LENGTHS: [usize; 16] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64];

struct Subscriber {
    channel: u8,
    filter: CanFilter,
    tx: mpsc::Sender<CanFrame>,
    dropped: Arc<AtomicU64>,
}

#[derive(Default)]
struct Registry {
    subscribers: StdMutex<Vec<Subscriber>>,
    failed: AtomicBool,
}

impl Registry {
    fn fail(&self) {
        self.failed.store(true, Ordering::Release);
        self.subscribers.lock().unwrap().clear();
    }

    fn dispatch(&self, channel: u8, frame: CanFrame) {
        self.subscribers.lock().unwrap().retain(|sub| {
            if sub.tx.is_closed() {
                return false;
            }
            if sub.channel == channel && sub.filter.matches(&frame) {
                if let Err(mpsc::error::TrySendError::Full(_)) = sub.tx.try_send(frame.clone()) {
                    sub.dropped.fetch_add(1, Ordering::Relaxed);
                }
            }
            true
        });
    }
}

struct SendRequest {
    bytes: Vec<u8>,
    reply: oneshot::Sender<Result<(), CanIoError>>,
}

struct SharedUsb {
    tx: mpsc::UnboundedSender<SendRequest>,
    echo: AtomicU32,
    registry: Arc<Registry>,
    reader: tokio::task::JoinHandle<()>,
    sender: tokio::task::JoinHandle<()>,
    _interface: nusb::Interface,
    _device: nusb::Device,
}

impl Drop for SharedUsb {
    fn drop(&mut self) {
        self.registry.fail();
        self.reader.abort();
        self.sender.abort();
    }
}

struct ChannelBus {
    shared: Arc<SharedUsb>,
    channel: u8,
    config: GsUsbConfig,
    link: CanLinkConfig,
}

pub async fn open(configs: [GsUsbConfig; 2]) -> Result<[Arc<dyn CanBus>; 2]> {
    if configs[0].channel == configs[1].channel || configs.iter().any(|c| c.channel > 255) {
        bail!("Select two different USB CAN channels (0..255)");
    }
    let info = nusb::list_devices()
        .await?
        .find(|d| KNOWN_IDS.contains(&(d.vendor_id(), d.product_id())))
        .context("No gs_usb / candleLight adapter found")?;
    let device = info.open().await?;
    let interface = device.detach_and_claim_interface(0).await?;
    control_out(&interface, 0, 1, &0x0000_beefu32.to_le_bytes()).await?;
    // DEVICE_CONFIG: reserved[3], icount (last channel index), sw, hw.
    let device_config = control_in(&interface, 5, 0, 12).await?;
    if device_config.len() < 4
        || configs
            .iter()
            .any(|c| c.channel > u16::from(device_config[3]))
    {
        bail!("Selected CAN channel is not present on this USB adapter");
    }
    let mut links = Vec::new();
    for config in &configs {
        let bt = control_in(&interface, 4, config.channel, 40).await?;
        if bt.len() < 8 || u32::from_le_bytes(bt[4..8].try_into().unwrap()) != 80_000_000 {
            bail!("Dual CAN timing profiles require a verified 80 MHz CAN clock");
        }
        let features = u32::from_le_bytes(bt[0..4].try_into().unwrap());
        if config.fd && (features & (1 << 8) == 0 || config.data.is_none()) {
            bail!(
                "CAN{} does not support the requested CAN FD profile",
                config.channel
            );
        }
        // Reconnect may inherit a running channel. Reset before changing timing.
        control_out(&interface, 2, config.channel, &[0; 8]).await?;
        control_out(&interface, 1, config.channel, &timing_bytes(config.nominal)).await?;
        if let Some(data) = config.data.filter(|_| config.fd) {
            control_out(&interface, 10, config.channel, &timing_bytes(data)).await?;
        }
        links.push(CanLinkConfig {
            fd_enabled: Some(config.fd),
            nominal: Some(bit_timing(config.nominal)),
            data: config.data.filter(|_| config.fd).map(bit_timing),
        });
    }
    for config in &configs {
        let mut mode = [0u8; 8];
        mode[..4].copy_from_slice(&1u32.to_le_bytes());
        mode[4..].copy_from_slice(&(if config.fd { 1u32 << 8 } else { 0 }).to_le_bytes());
        control_out(&interface, 2, config.channel, &mode).await?;
    }
    let in_ep = interface.endpoint::<Bulk, In>(0x81)?;
    let out_ep = interface.endpoint::<Bulk, Out>(0x01)?;
    let registry = Arc::new(Registry::default());
    let (tx, rx) = mpsc::unbounded_channel();
    let shared = Arc::new(SharedUsb {
        tx,
        echo: AtomicU32::new(0),
        reader: tokio::spawn(read_loop(in_ep, registry.clone())),
        sender: tokio::spawn(send_loop(out_ep, rx, registry.clone())),
        registry,
        _interface: interface,
        _device: device,
    });
    Ok(std::array::from_fn(|i| {
        Arc::new(ChannelBus {
            shared: shared.clone(),
            channel: configs[i].channel as u8,
            config: configs[i],
            link: links[i],
        }) as Arc<dyn CanBus>
    }))
}

async fn control_out(iface: &nusb::Interface, request: u8, value: u16, data: &[u8]) -> Result<()> {
    iface
        .control_out(
            ControlOut {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index: 0,
                data,
            },
            USB_TIMEOUT,
        )
        .await?;
    Ok(())
}

async fn control_in(
    iface: &nusb::Interface,
    request: u8,
    value: u16,
    length: u16,
) -> Result<Vec<u8>> {
    Ok(iface
        .control_in(
            ControlIn {
                control_type: ControlType::Vendor,
                recipient: Recipient::Interface,
                request,
                value,
                index: 0,
                length,
            },
            USB_TIMEOUT,
        )
        .await?)
}

fn timing_bytes(t: GsTiming) -> Vec<u8> {
    [t.prop_seg, t.phase_seg1, t.phase_seg2, t.sjw, t.brp]
        .into_iter()
        .flat_map(u32::to_le_bytes)
        .collect()
}

fn bit_timing(t: GsTiming) -> CanBitTiming {
    let total = 1 + t.prop_seg + t.phase_seg1 + t.phase_seg2;
    CanBitTiming {
        bitrate: Some(80_000_000 / (total * t.brp)),
        sample_point_per_mille: Some(((1 + t.prop_seg + t.phase_seg1) * 1000 / total) as u16),
    }
}

// One sender owns USB completions even if the IPC caller is cancelled. A
// timeout poisons BOTH channels: their endpoint is shared, so neither may
// consume a late completion from a previous submission.
async fn send_loop(
    mut ep: nusb::Endpoint<Bulk, Out>,
    mut rx: mpsc::UnboundedReceiver<SendRequest>,
    registry: Arc<Registry>,
) {
    while let Some(req) = rx.recv().await {
        if req.reply.is_closed() {
            continue;
        }
        if registry.failed.load(Ordering::Acquire) {
            let _ = req.reply.send(Err(CanIoError::Disconnected));
            continue;
        }
        ep.submit(Buffer::from(req.bytes));
        let result = match tokio::time::timeout(USB_TIMEOUT, ep.next_complete()).await {
            Ok(done) => done.status.map_err(CanIoError::backend),
            Err(_) => Err(CanIoError::Disconnected),
        };
        if result.is_err() {
            registry.fail();
        }
        let _ = req.reply.send(result);
    }
}

async fn read_loop(mut ep: nusb::Endpoint<Bulk, In>, registry: Arc<Registry>) {
    for _ in 0..8 {
        ep.submit(Buffer::new(512));
    }
    loop {
        let done = ep.next_complete().await;
        if done.status.is_err() {
            registry.fail();
            return;
        }
        if let Some((channel, frame)) = decode(&done.buffer[..done.actual_len]) {
            registry.dispatch(channel, frame);
        }
        ep.submit(Buffer::new(512));
    }
}

fn decode(bytes: &[u8]) -> Option<(u8, CanFrame)> {
    if bytes.len() < 12 || bytes[..4] != u32::MAX.to_le_bytes() {
        return None;
    }
    let raw = u32::from_le_bytes(bytes[4..8].try_into().ok()?);
    if raw & 0x2000_0000 != 0 {
        return None;
    }
    let id = if raw & 0x8000_0000 != 0 {
        CanId::Extended(raw & 0x1fff_ffff)
    } else {
        CanId::Standard((raw & 0x7ff) as u16)
    };
    let fd = bytes[10] & 2 != 0;
    let dlc = bytes[8] as usize;
    if dlc > 15 || (!fd && dlc > 8) {
        return None;
    }
    let frame = if raw & 0x4000_0000 != 0 {
        CanFrame::new_remote(id, dlc as u8).ok()?
    } else {
        let payload = bytes.get(12..12 + DLC_LENGTHS[dlc])?;
        if fd {
            CanFrame::new_fd(id, payload, bytes[10] & 4 != 0).ok()?
        } else {
            CanFrame::new_data(id, payload).ok()?
        }
    };
    Some((bytes[9], frame))
}

fn encode(frame: &CanFrame, echo: u32, config: GsUsbConfig) -> Result<Vec<u8>, CanIoError> {
    let fd = matches!(frame.kind(), FrameKind::Fd { .. });
    if fd && !config.fd {
        return Err(CanIoError::Disconnected);
    }
    let mut bytes = vec![0; if config.fd { 76 } else { 20 }];
    bytes[..4].copy_from_slice(&echo.to_le_bytes());
    let mut raw = frame.id().raw();
    if frame.id().is_extended() {
        raw |= 0x8000_0000;
    }
    if matches!(frame.kind(), FrameKind::Remote) {
        raw |= 0x4000_0000;
    }
    bytes[4..8].copy_from_slice(&raw.to_le_bytes());
    bytes[8] = if fd {
        DLC_LENGTHS
            .iter()
            .position(|n| *n >= frame.data().len())
            .unwrap() as u8
    } else {
        frame.dlc() as u8
    };
    bytes[9] = config.channel as u8;
    bytes[10] = match frame.kind() {
        FrameKind::Fd { brs } => 2 | if brs { 4 } else { 0 },
        _ => 0,
    };
    if !matches!(frame.kind(), FrameKind::Remote) {
        bytes[12..12 + frame.data().len()].copy_from_slice(frame.data());
    }
    Ok(bytes)
}

#[async_trait]
impl CanBus for ChannelBus {
    async fn send(&self, frame: CanFrame) -> Result<(), CanIoError> {
        if self.shared.registry.failed.load(Ordering::Acquire) {
            return Err(CanIoError::Disconnected);
        }
        let echo = self.shared.echo.fetch_add(1, Ordering::Relaxed) & 0x7fff_ffff;
        let bytes = encode(&frame, echo, self.config)?;
        let (reply, rx) = oneshot::channel();
        self.shared
            .tx
            .send(SendRequest { bytes, reply })
            .map_err(|_| CanIoError::Disconnected)?;
        rx.await.unwrap_or(Err(CanIoError::Disconnected))
    }

    async fn subscribe(&self, filter: CanFilter) -> Result<Box<dyn CanRx>, CanIoError> {
        let (tx, rx) = mpsc::channel(512);
        let dropped = Arc::new(AtomicU64::new(0));
        let mut subs = self.shared.registry.subscribers.lock().unwrap();
        if self.shared.registry.failed.load(Ordering::Acquire) {
            return Err(CanIoError::Disconnected);
        }
        subs.push(Subscriber {
            channel: self.channel,
            filter,
            tx,
            dropped: dropped.clone(),
        });
        Ok(Box::new(ChannelRx {
            rx,
            dropped,
            _shared: self.shared.clone(),
        }))
    }

    fn capabilities(&self) -> CanCapabilities {
        CanCapabilities {
            fd: self.config.fd,
            max_dlen: if self.config.fd { 64 } else { 8 },
        }
    }

    async fn link_config(&self) -> Result<Option<CanLinkConfig>, CanIoError> {
        Ok(Some(self.link))
    }
}

struct ChannelRx {
    rx: mpsc::Receiver<CanFrame>,
    dropped: Arc<AtomicU64>,
    _shared: Arc<SharedUsb>,
}

#[async_trait]
impl CanRx for ChannelRx {
    async fn recv(&mut self) -> Result<CanFrame, CanIoError> {
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        if dropped != 0 {
            return Err(CanIoError::Lagged { dropped });
        }
        self.rx.recv().await.ok_or(CanIoError::Disconnected)
    }
    fn try_recv(&mut self) -> Result<Option<CanFrame>, CanIoError> {
        let dropped = self.dropped.swap(0, Ordering::Relaxed);
        if dropped != 0 {
            return Err(CanIoError::Lagged { dropped });
        }
        match self.rx.try_recv() {
            Ok(f) => Ok(Some(f)),
            Err(mpsc::error::TryRecvError::Empty) => Ok(None),
            Err(_) => Err(CanIoError::Disconnected),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mixed_fd_and_classic_frames_retain_channel_and_payload() {
        for (config, frame) in [
            (
                GsUsbConfig::fd_1m_5m(),
                CanFrame::new_fd(CanId::Standard(0x181), &[7; 12], true).unwrap(),
            ),
            (
                GsUsbConfig::classic_1m().with_channel(1),
                CanFrame::new_data(CanId::Standard(0x181), &[8; 8]).unwrap(),
            ),
        ] {
            let bytes = encode(&frame, u32::MAX, config).unwrap();
            let (ch, decoded) = decode(&bytes).unwrap();
            assert_eq!(ch, config.channel as u8);
            assert_eq!(decoded.data(), frame.data());
            assert_eq!(decoded.kind(), frame.kind());
            assert!(decode(&bytes[..13]).is_none());
            assert!(decode(&encode(&frame, 10, config).unwrap()).is_none());
        }
    }

    #[tokio::test]
    async fn identical_can_ids_are_routed_only_to_their_own_channel() {
        let registry = Registry::default();
        let (tx0, mut rx0) = mpsc::channel(2);
        let (tx1, mut rx1) = mpsc::channel(2);
        for (channel, tx) in [(0, tx0), (1, tx1)] {
            registry.subscribers.lock().unwrap().push(Subscriber {
                channel,
                tx,
                filter: CanFilter::exact_standard(0x181),
                dropped: Arc::new(AtomicU64::new(0)),
            });
        }
        registry.dispatch(1, CanFrame::new_data(CanId::Standard(0x181), &[1]).unwrap());
        assert!(rx0.try_recv().is_err());
        assert_eq!(rx1.try_recv().unwrap().data(), &[1]);
        registry.fail();
        assert!(rx0.recv().await.is_none());
        assert!(rx1.recv().await.is_none());
    }
}
