#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(
    clippy::missing_safety_doc,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap
)]

use core::cmp::Ordering;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_net::{Config, Ipv4Address, Ipv4Cidr, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::ledc::{
    Ledc, LowSpeed,
    channel::{self, ChannelIFace},
    timer::{self, TimerIFace},
};
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_radio::wifi::{
    AccessPointConfig, AuthMethod, Config as WifiDriverConfig, ModeConfig, WifiController,
    WifiDevice, WifiEvent,
};

use esp_backtrace as _;
use esp_println as _;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = "FSE-Tsunderacer";
const PASSWORD: &str = "cirnobaka9";
const LISTEN_PORT: u16 = 9999;
const STATIC_IP: Ipv4Address = Ipv4Address::new(192, 168, 9, 9);
const GATEWAY_IP: Ipv4Address = Ipv4Address::new(192, 168, 9, 1);

// Servo Math (SG90 @ 50Hz)
const SERVO_FREQ: u32 = 50;
const SERVO_PERIOD_MS: u32 = 1000 / SERVO_FREQ;
const MAX_DUTY: u16 = 8191;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: StaticCell<$t> = StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.init($val);
        x
    }};
}

fn calculate_servo_duty(angle_byte: i8, max_duty: u16) -> u16 {
    let min_pulse_ms = 1.0;
    let max_pulse_ms = 2.0;

    let input = f32::from(angle_byte).clamp(-100.0, 100.0);

    let normalized = (input - (-100.0)) / (100.0 - (-100.0));

    let pulse_ms = min_pulse_ms + (normalized * (max_pulse_ms - min_pulse_ms));

    #[allow(clippy::cast_precision_loss)]
    let duty = (pulse_ms / (SERVO_PERIOD_MS as f32)) * f32::from(max_duty);

    duty as u16
}

fn calculate_motor_duty(val: u8, max_duty: u16) -> u16 {
    let percent = f32::from(val) / 100.0;
    (percent * f32::from(max_duty)) as u16
}

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    info!("Starting Wi-Fi AP task");
    loop {
        if !matches!(controller.is_started(), Ok(true)) {
            let ap_config = ModeConfig::AccessPoint(
                AccessPointConfig::default()
                    .with_ssid(SSID.into())
                    .with_password(PASSWORD.into())
                    .with_auth_method(AuthMethod::Wpa2Personal),
            );

            controller.set_config(&ap_config).unwrap();
            info!("Starting Wi-Fi controller in AP Mode...");
            controller.start().unwrap();
        }

        controller.wait_for_event(WifiEvent::ApStop).await;
        warn!("Wi-Fi AP Stopped! Restarting...");
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, WifiDevice<'static>>) {
    runner.run().await;
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);

    let rng = Rng::new();
    let seed = u64::from(rng.random());

    let sw_ints =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    info!("Initializing UNSAFETY-CAR...");

    let mut ledc = Ledc::new(peripherals.LEDC);

    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);

    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty13Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50),
        })
        .unwrap();

    // Channel 0: Servo (GPIO 4)
    let mut servo_channel = ledc.channel(channel::Number::Channel0, peripherals.GPIO4);

    servo_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 7,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // Channel 1: Front Motor (GPIO 3)
    let mut front_channel = ledc.channel(channel::Number::Channel1, peripherals.GPIO3);
    front_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // Channel 2: Back Motor (GPIO 2)
    let mut back_channel = ledc.channel(channel::Number::Channel2, peripherals.GPIO2);
    back_channel
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    info!("PWM Configured. Max Duty: {}", MAX_DUTY);

    // --- 2. Init Wi-Fi & Network ---
    let init = mk_static!(esp_radio::Controller<'static>, esp_radio::init().unwrap());
    let (controller, interfaces) =
        esp_radio::wifi::new(init, peripherals.WIFI, WifiDriverConfig::default()).unwrap();

    let wifi_interface = interfaces.ap;

    let net_config = Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(STATIC_IP, 24),
        gateway: Some(GATEWAY_IP),
        #[allow(clippy::default_trait_access)]
        dns_servers: Default::default(),
    });

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        net_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection(controller)).unwrap();
    spawner.spawn(net_task(runner)).unwrap();

    info!("Network Stack Initialized. IP: {}", STATIC_IP);

    // --- 3. UDP Loop ---
    let mut rx_buffer = [0; 128];
    let mut tx_buffer = [0; 128];
    let mut rx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 4];
    let mut tx_meta = [embassy_net::udp::PacketMetadata::EMPTY; 4];

    let mut socket = embassy_net::udp::UdpSocket::new(
        stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    if let Err(e) = socket.bind(LISTEN_PORT) {
        error!("Failed to bind UDP socket: {:?}", e);
        panic!("UDP Bind Failed");
    }

    info!("UDP Listening on port {}", LISTEN_PORT);

    let mut buf = [0u8; 64];

    loop {
        match socket.recv_from(&mut buf).await {
            Ok((size, _endpoint)) => {
                if size == 2 {
                    let throttle_byte = buf[0] as i8;
                    let steer_byte = buf[1] as i8;

                    info!("T: {}, S: {}", throttle_byte, steer_byte);

                    // 1. Steering
                    let servo_duty = calculate_servo_duty(steer_byte, MAX_DUTY);
                    let _ = servo_channel.set_duty_cycle(servo_duty);

                    // 2. Throttle
                    match throttle_byte.cmp(&0) {
                        Ordering::Greater => {
                            // Forward
                            let duty = calculate_motor_duty(throttle_byte as u8, MAX_DUTY);
                            let _ = back_channel.set_duty_cycle(0);
                            let _ = front_channel.set_duty_cycle(duty);
                        }
                        Ordering::Less => {
                            // Reverse
                            let duty = calculate_motor_duty(throttle_byte.unsigned_abs(), MAX_DUTY);
                            let _ = front_channel.set_duty_cycle(0);
                            let _ = back_channel.set_duty_cycle(duty);
                        }
                        Ordering::Equal => {
                            // Stop
                            let _ = front_channel.set_duty_cycle(0);
                            let _ = back_channel.set_duty_cycle(0);
                        }
                    }
                } else {
                    warn!("Invalid packet size: {}", size);
                }
            }
            Err(e) => {
                warn!("UDP Receive Error: {:?}", e);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}
