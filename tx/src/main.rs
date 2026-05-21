#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};
use esp_csi_rs::logging::logging::LogMode;
use esp_csi_rs::{
    config::CsiConfig, logging::logging::init_logger, CSINode, CollectionMode, EspNowConfig,
};
use esp_csi_rs::{get_pps_tx, get_total_tx_packets, log_ln, CSINodeClient, CSINodeHardware};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::WifiController;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

static WIFI_CONTROLLER: static_cell::StaticCell<WifiController<'static>> =
    static_cell::StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

async fn stats_task() {
    loop {
        Timer::after_secs(1).await;
        log_ln!("TX PPS: {:#?}, TX Total: {:#?}", get_pps_tx(), get_total_tx_packets());
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    init_logger(spawner, LogMode::Text);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 61440);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    log_ln!("TX Central booting up");

    let config_radio = esp_radio::wifi::ControllerConfig::default();
    let (wifi_controller, mut interfaces) = esp_radio::wifi::new(peripherals.WIFI, config_radio)
        .expect("Failed to initialize Wi-Fi controller");

    let controller = WIFI_CONTROLLER.init(wifi_controller);

    let mut node_handle = CSINodeClient::new();
    let csi_hardware = CSINodeHardware::new(&mut interfaces, controller);
    // Use Central role with ESP‑NOW at 10 Hz traffic frequency
    let mut node = CSINode::new(
        esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::EspNow(EspNowConfig::default())),
        CollectionMode::Listener,
        Some(CsiConfig::default()),
        Some(10), // traffic_freq_hz = 10 Hz
        csi_hardware,
    );
    node.set_protocol(esp_radio::wifi::Protocol::N);
    node.set_rate(esp_radio::esp_now::WifiPhyRate::RateMcs0Lgi);

    join(node.run(), stats_task()).await;

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
