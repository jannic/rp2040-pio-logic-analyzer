#![no_std]
#![no_main]

use core::iter::repeat;
use panic_probe as _;

use cortex_m_rt::entry;
use hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{FunctionPio0, Pin, Pins},
    pac,
    pio::PIOExt,
    sio::Sio,
    timer::Timer,
    watchdog::Watchdog,
};
use rp2040_hal as hal;

use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_pio::Ws2812;

use rp2040_pio_logic_analyzer as probe;
use rtt_target::*;

fn init_rtt() -> UpChannel {
    let channels = rtt_init! {
    up: {
        0: {
        size: 1024
        mode: NoBlockSkip
        name: "Terminal"
        }
        1: {
        size: 1024
        mode: BlockIfFull
        name: "Data"
        }
    }
    down: {
        0: {
        size: 16
        name: "Terminal"
        }
    }
    };
    set_print_channel(channels.up.0);
    channels.up.1
}

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[entry]
fn main() -> ! {
    let mut channel = init_rtt();

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let external_xtal_freq_hz = 12_000_000u32;

    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let led: Pin<_, FunctionPio0> = pins.gpio2.into_mode();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let mut ws = Ws2812::new(
        led,
        &mut pio,
        sm0,
        clocks.system_clock.freq(),
        timer.count_down(),
    );

    let mut dma_buf = [0u32; 8 * 1024];

    let colors: RGB8 = (0xf0u8, 0x55u8, 0x0fu8).into();
    probe::run_with_logic_analyzer(pac.PIO1, pac.DMA, &mut pac.RESETS, &mut dma_buf, || {
        ws.write(repeat(colors).take(3)).unwrap();
        // make sure PIO finishes to write out ws2812 data
        // (This is only necessary because ws.write() doesn't block but
        // returns immediately, while the transfer is still in progress.)
        delay.delay_us(150);
    });

    probe::write_dma_buffer(&mut channel, &dma_buf[..]);

    #[allow(clippy::empty_loop)]
    loop {}
}
