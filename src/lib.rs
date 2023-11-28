#![no_std]
#![feature(type_alias_impl_trait)]

use defmt::{info, warn, Debug2Format};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use embassy_time::Timer;
use rand::Rng;
use static_cell::make_static;

// need one more socket for DHCP
pub const WEB_TASK_POOL_SIZE: usize = 8;
const N_SOCKETS: usize = WEB_TASK_POOL_SIZE + 1;

embassy_rp::bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<embassy_rp::peripherals::PIO0>;
});

pub async fn start_wifi(
    spawner: embassy_executor::Spawner,
    p_pwr: PIN_23,
    p_cs: PIN_25,
    p_pio: PIO0,
    p_dio: PIN_24,
    p_clk: PIN_29,
    p_dma: DMA_CH0,
    wifi_ssid: &str,
    wifi_pass: &str,
) -> (
    &'static embassy_net::Stack<cyw43::NetDriver<'static>>,
    cyw43::Control<'static>,
) {
    // start WIFI
    info!("Initialize net stack...");
    let fw = include_bytes!("../cyw43-firmware/43439A0.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0.bin --format bin --chip RP2040 --base-address 0x10100000
    // let fw = unsafe { core::slice::from_raw_parts(0x10100000 as *const u8, 230321) };

    let pwr = Output::new(p_pwr, Level::Low);
    let cs = Output::new(p_cs, Level::High);
    let mut pio = Pio::new(p_pio, Irqs);
    let net_spi =
        cyw43_pio::PioSpi::new(&mut pio.common, pio.sm0, pio.irq0, cs, p_dio, p_clk, p_dma);

    let (net_device, mut control, runner) =
        cyw43::new(make_static!(cyw43::State::new()), pwr, net_spi, fw).await;
    let stack = embassy_net::Stack::new(
        net_device,
        embassy_net::Config::dhcpv4(Default::default()),
        make_static!(embassy_net::StackResources::<N_SOCKETS>::new()),
        embassy_rp::clocks::RoscRng.gen(),
    );

    // wifi_task must be run before initializing control
    spawner.must_spawn(wifi_task(runner));

    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    // To make flashing faster for development, you may want to flash the firmwares independently
    // at hardcoded addresses, instead of baking them into the program with `include_bytes!`:
    //     probe-rs download 43439A0_clm.bin --format bin --chip RP2040 --base-address 0x10140000
    // let clm = unsafe { core::slice::from_raw_parts(0x10140000 as *const u8, 4752) };
    control.init(clm).await;

    let stack = make_static!(stack);
    spawner.must_spawn(net_task(stack));
    // info!("Net stack ok");

    // info!("Join WIFI {}...", wifi_ssid);
    while let Err(e) = control.join_wpa2(wifi_ssid, wifi_pass).await {
        warn!("Could not join WIFI {}: {}", wifi_ssid, Debug2Format(&e));
    }
    // info!("WIFI connected");

    // Wait for DHCP, not necessary when using static IP
    // info!("Waiting for DHCP...");
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }

    (stack, control)
}

pub struct EmbassyTimer;

impl picoserve::Timer for EmbassyTimer {
    type Duration = embassy_time::Duration;
    type TimeoutError = embassy_time::TimeoutError;

    async fn run_with_timeout<F: core::future::Future>(
        &mut self,
        duration: Self::Duration,
        future: F,
    ) -> Result<F::Output, Self::TimeoutError> {
        embassy_time::with_timeout(duration, future).await
    }
}

#[macro_export]
macro_rules! picoserve_task {
    ($AppRouter:ty, $State:ty) => {
        #[embassy_executor::task]
        async fn picoserve_task(
            id: usize,
            stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>,
            app: &'static picoserve::Router<$AppRouter, $State>,
            config: &'static picoserve::Config<Duration>,
            state: $State,
        ) {
            let mut rx_buffer = [0; 1024];
            let mut tx_buffer = [0; 1024];

            loop {
                let mut socket =
                    embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

                if let Err(e) = socket.accept(80).await {
                    warn!("{}: accept error: {:?}", id, e);
                    continue;
                }

                let (socket_rx, socket_tx) = socket.split();

                if let Err(err) = picoserve::serve_with_state(
                    app,
                    embassy_rp_wifi::EmbassyTimer,
                    config,
                    &mut [0; 2048],
                    socket_rx,
                    socket_tx,
                    &state,
                )
                .await
                {
                    defmt::error!("{}", Debug2Format(&err));
                }
            }
        }
    };
}

#[embassy_executor::task]
pub async fn wifi_task(
    runner: cyw43::Runner<
        'static,
        Output<'static, PIN_23>,
        cyw43_pio::PioSpi<'static, PIN_25, PIO0, 0, DMA_CH0>,
    >,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(stack: &'static embassy_net::Stack<cyw43::NetDriver<'static>>) -> ! {
    stack.run().await
}
