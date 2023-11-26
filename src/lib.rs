#![no_std]
#![feature(type_alias_impl_trait)]

use defmt::{info, warn, Debug2Format};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIN_23, PIN_24, PIN_25, PIN_29, PIO0};
use embassy_rp::pio::Pio;
use rand::Rng;
use static_cell::make_static;

pub const WEB_TASK_POOL_SIZE: usize = 8;

// need one more socket for DHCP
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
    info!("Net stack ok");

    (stack, control)
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
