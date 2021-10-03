//! Internal logic analyizer for the RP2040
//!
//! This crate allows to use a PIO to periodically read out the state of all
//! GPIO pins, implementing a simple logic analyzer
#![no_std]

use core::fmt::Write;
use rp2040_hal as hal;

use hal::{pac, pio::PIOExt, pio::Rx, pio::Tx, pio::ValidStateMachine};

/// Write the contents of dma_buf to writer, in VCD format, readable
/// (for example) with pulseview.
pub fn write_dma_buffer<W: Write>(writer: &mut W, dma_buf: &[u32]) {
    // timescale: 5 PIO steps / 125MHz => 40ns
    writeln!(writer, "$timescale 40 ns $end").unwrap();
    for w in 0..30 {
        writeln!(writer, "$var wire 1 {} GPIO{} $end", (b'!' + w) as char, w).unwrap();
    }
    writeln!(writer, "$upscope $end").unwrap();
    writeln!(writer, "$enddefinitions $end").unwrap();
    let start = !dma_buf[0];
    let mut prev = 0;
    let mut last_time = start;
    dma_buf.chunks(2).filter(|c| c[0] != 0).for_each(|c| {
        writeln!(writer, "#{}", (!c[0]) - start).unwrap();
        let this = c[1];
        for w in 0..30 {
            let this = this >> w & 1;
            let prev = prev >> w & 1;
            if this != prev {
                writeln!(writer, "{} {}", this, (b'!' + w) as char).unwrap();
            }
        }
        prev = this;
        last_time = !c[0];
    });
    // add 100µs of quiet time to help pulseview decoders
    writeln!(writer, "#{}", last_time - start + 2500).unwrap();
}

pub trait PioDreq {
    fn dreq_base() -> u8;
}

impl PioDreq for pac::PIO0 {
    fn dreq_base() -> u8 {
        0
    }
}
impl PioDreq for pac::PIO1 {
    fn dreq_base() -> u8 {
        8
    }
}

trait Dreq {
    fn dreq(&self) -> u8;
}

impl<P: PioDreq, SM: ValidStateMachine<PIO = P>> Dreq for Rx<SM> {
    fn dreq(&self) -> u8 {
        SM::PIO::dreq_base() + SM::id() as u8 + 4
    }
}
impl<P: PioDreq, SM: ValidStateMachine<PIO = P>> Dreq for Tx<SM> {
    fn dreq(&self) -> u8 {
        SM::PIO::dreq_base() + SM::id() as u8
    }
}

pub fn run_with_logic_analyzer<T, R, PIO>(
    pio0: PIO,
    dma: pac::DMA,
    resets: &mut pac::RESETS,
    dma_buf: &mut [u32],
    run: T,
) -> R
where
    T: FnOnce() -> R,
    PIO: PIOExt + PioDreq,
{
    // Make sure DMA is out of reset
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    // prepare PIO
    let program = pio_proc::pio!(
        pio::RP2040_MAX_PROGRAM_SIZE,
        "
            mov y, null
          .wrap_target
          loop: ; 5 cycles per loop
            mov x, pins [1]
            jmp x!=y change
            jmp loop [1]
          change:
            in x, 32
            mov y,x
          .wrap
        "
    );
    let program2 = pio_proc::pio!(
        pio::RP2040_MAX_PROGRAM_SIZE,
        "
          .wrap_target
          loop: ; 5 cycles per loop
            jmp !osre data
            jmp next [2]
          data:
            in x, 32
            out isr, 32
            push
          next:
            jmp x-- loop
          .wrap
        "
    );

    let rxf_ptr = core::ptr::addr_of!(pio0.rxf);
    let rxf1_ptr = core::ptr::addr_of!(pio0.rxf[1]);
    let txf1_ptr = core::ptr::addr_of!(pio0.txf[1]);

    let div = 1f32;

    let (mut pio, sm0, sm1, sm2, sm3) = pio0.split(resets);
    let installed = pio.install(&program2.program).unwrap();
    let (pio_sm1, rx1, tx1) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .clock_divisor(div)
        .autopull(true)
        .autopush(true)
        .build(sm1);

    let installed = pio.install(&program.program).unwrap();
    // TODO:
    // - stop SM
    // - empty RX buffer
    // - start DMA
    // - start SM
    let (pio_sm0, rx0, tx0) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_pin_base(0)
        .clock_divisor(div)
        .autopush(true)
        .in_shift_direction(rp2040_hal::pio::ShiftDirection::Left)
        .build(sm0);

    unsafe {
        dma.chan_abort.write(|w| w.bits(3));
        while dma.chan_abort.read().bits() & 3 != 0 {}

        // (time, data) sm1 -> dma_buf
        let dma_buf_addr = core::ptr::addr_of!(dma_buf[0]);
        dma.ch[0]
            .ch_write_addr
            .write(|w| w.bits(dma_buf_addr as u32));
        dma.ch[0].ch_read_addr.write(|w| w.bits(rxf1_ptr as u32));
        //dma.ch[0].ch_read_addr.write(|w| w.bits(t_ptr as u32));
        dma.ch[0]
            .ch_trans_count
            .write(|w| w.bits(dma_buf.len() as u32));
        dma.ch[0].ch_ctrl_trig.write(|w| {
            w.data_size()
                .size_word()
                .incr_write()
                .bit(true)
                .treq_sel()
                .bits(rx1.dreq())
                .en()
                .bit(true)
        });
        // data sm0 -> sm1
        dma.ch[1].ch_write_addr.write(|w| w.bits(txf1_ptr as u32));
        dma.ch[1].ch_read_addr.write(|w| w.bits(rxf_ptr as u32));
        dma.ch[1]
            .ch_trans_count
            .write(|w| w.bits(dma_buf.len() as u32 / 2));
        dma.ch[1].ch_ctrl_trig.write(|w| {
            w.data_size()
                .size_word()
                .treq_sel()
                .bits(rx0.dreq())
                .en()
                .bit(true)
        });
    };
    let pio_sm1 = pio_sm1.start();
    let pio_sm0 = pio_sm0.start();
    let result = run();

    // Release PIO
    let (sm0, installed) = pio_sm0.uninit(rx0, tx0);
    pio.uninstall(installed);
    let (sm1, installed) = pio_sm1.uninit(rx1, tx1);
    pio.uninstall(installed);
    pio.free(sm0, sm1, sm2, sm3);

    dma.ch[0].ch_ctrl_trig.write(|w| w.en().bit(false));
    dma.ch[1].ch_ctrl_trig.write(|w| w.en().bit(false));
    dma.chan_abort.write(|w| unsafe { w.bits(3) });
    while dma.chan_abort.read().bits() & 3 != 0 {}
    result
}
