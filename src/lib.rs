//! Internal logic analyizer for the RP2040
//!
//! This crate allows to use a PIO to periodically read out the state of all
//! GPIO pins, implementing a simple logic analyzer
#![no_std]

use core::fmt::Write;
use rp2040_hal as hal;

use hal::{pac, pio::PIOExt};

pub fn write_header<W: Write>(writer: &mut W) {
    // timescale: 5 PIO steps / 125MHz => 40ns
    writeln!(writer, "$timescale 40 ns $end").unwrap();
    for w in 0..30 {
        writeln!(writer, "$var wire 1 {} GPIO{} $end", (b'!' + w) as char, w).unwrap();
    }
    writeln!(writer, "$upscope $end").unwrap();
    writeln!(writer, "$enddefinitions $end").unwrap();
}

fn write_footer<W: Write>(writer: &mut W, time: u32) {
    // Write another time stamp, 100µs later, to help pulseview decoders
    writeln!(writer, "#{}", time + 2500).unwrap();
}

pub fn write_data<W: Write>(writer: &mut W, time: u32, data: u32, prev_data: u32) {
    // TODO: no unwraps!
    writeln!(writer, "#{}", time).unwrap(); // TODO remove unwrap
    for w in 0..30 {
        let this = data >> w & 1;
        let prev = prev_data >> w & 1;
        if this != prev {
            writeln!(writer, "{} {}", this, (b'!' + w) as char).unwrap(); // TODO remove unwrap
        }
    }
}

/// Write the contents of dma_buf to writer, in VCD format, readable
/// (for example) with pulseview.
pub fn write_dma_buffer<W: Write>(writer: &mut W, dma_buf: &[u32]) {
    write_header(writer);
    let start = !dma_buf[0];
    let mut prev = 0;
    let mut last_time = start;
    dma_buf.chunks(2).filter(|c| c[0] != 0).for_each(|c| {
        let time = (!c[0]) - start;
        let data = c[1];
        write_data(writer, time, data, prev);
        prev = data;
        last_time = !c[0];
    });
    write_footer(writer, last_time - start);
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
    PIO: PIOExt,
{
    run_with_logic_analyzer2(pio0, dma, resets, dma_buf, |_| run())
}

struct DataAccessorInner<'a> {
    dma_buf: &'a mut [u32],
    read_ptr: usize,
    dma: &'a pac::DMA,
}
pub struct DataAccessor<'a> {
    inner: DataAccessorInner<'a>,
}
pub struct Sample {
    pub data: u32,
    pub time: u32,
}
impl<'a> DataAccessor<'a> {
    pub fn try_next(&mut self) -> Option<Sample> {
        let dma_buf_addr = core::ptr::addr_of!(self.inner.dma_buf[0]) as u32;
        let mut dma_write_addr = self.inner.dma.ch[0].ch_write_addr.read().bits();
        while dma_write_addr < dma_buf_addr || dma_write_addr > dma_buf_addr + 0x8000 {
            // https://forums.raspberrypi.com/viewtopic.php?t=323813
            defmt::trace!(
                "dma_write_addr {=u32:x} < dma_buf_addr {=u32:x}",
                dma_write_addr,
                dma_buf_addr
            );
            dma_write_addr = self.inner.dma.ch[0].ch_write_addr.read().bits();
        }
        let dma_write_ptr = (dma_write_addr - dma_buf_addr) as usize / 4;
        let dma_len = self.inner.dma_buf.len();
        let dma_window_length = dma_len / 2;
        let dma_window_end = dma_write_ptr + dma_window_length; // may point outside of buffer!
                                                                // TODO: check this condition. covers all cases? off-by-one errors?

        // don't read from dma_write_ptr..(dma_write_ptr + dma_window_length), modulo dma_len.
        if (dma_write_ptr + 1..(dma_write_ptr + dma_window_length)).contains(&self.inner.read_ptr)
            || (dma_write_ptr..(dma_write_ptr + dma_window_length))
                .contains(&(self.inner.read_ptr + dma_len))
        // this handles the wrap-around case
        {
            //defmt::trace!("no new data; {=u32:x} {=u32:x} {=u32:x}", self.inner.read_ptr as u32, dma_write_ptr as u32, dma_window_end as u32);
            defmt::warn!(
                "Skipping! {=u32:x} {=u32:x} {=u32:x}",
                self.inner.read_ptr as u32,
                dma_write_ptr as u32,
                dma_window_end as u32
            );
            self.inner.read_ptr = dma_write_ptr & !1;
            defmt::warn!(
                "Updated: {=u32:x} {=u32:x} {=u32:x}",
                self.inner.read_ptr as u32,
                dma_write_ptr as u32,
                dma_window_end as u32
            );
            return None;
        }
        if self.inner.read_ptr == dma_write_ptr
            || self.inner.read_ptr + 1 == dma_write_ptr
            || self.inner.read_ptr + 2 == dma_write_ptr
        {
            // read ptr is immediately behind write ptr
            defmt::trace!("up to date @ {:x}", self.inner.read_ptr);
            return None;
        }
        defmt::trace!(
            "some new data; {=u32:x} {=u32:x} {=u32:x}",
            self.inner.read_ptr as u32,
            dma_write_ptr as u32,
            dma_window_end as u32
        );
        let c = &self.inner.dma_buf[self.inner.read_ptr..self.inner.read_ptr + 2];
        let time = !c[0];
        let data = c[1];
        /*
        let time =
            unsafe {
                core::ptr::read_volatile((dma_buf_addr as usize +
                self.inner.read_ptr) as *const u32)
            };
        let data =
            unsafe {
                core::ptr::read_volatile((dma_buf_addr as usize +
                self.inner.read_ptr + 1) as *const u32)
            };
        */
        self.inner.read_ptr += 2;
        if self.inner.read_ptr + 1 >= self.inner.dma_buf.len() {
            self.inner.read_ptr = 0;
        }
        Some(Sample { data, time })
    }

    pub fn dbg_ptr(&mut self) -> (usize, u32) {
        (
            self.inner.read_ptr,
            self.inner.dma.ch[0].ch_write_addr.read().bits(),
        )
    }
}

// TODO: Rename
pub fn run_with_logic_analyzer2<T, R, PIO>(
    pio0: PIO,
    dma: pac::DMA,
    resets: &mut pac::RESETS,
    dma_buf: &mut [u32],
    run: T,
) -> R
where
    T: FnOnce(&mut DataAccessor) -> R,
    PIO: PIOExt,
{
    // Make sure DMA is out of reset
    resets.reset.modify(|_, w| w.dma().set_bit());
    resets.reset.modify(|_, w| w.dma().clear_bit());
    while resets.reset_done.read().dma().bit_is_clear() {}

    // prepare PIO
    let program = pio_proc::pio_asm!(
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
    let program2 = pio_proc::pio_asm!(
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

    let ctrl: hal::pac::dma::ch::CH_CTRL_TRIG = unsafe { core::mem::transmute(0u32) };
    ctrl.write(|w| {
        unsafe {
            w.data_size()
                .size_word()
                .incr_write()
                .bit(true)
                .ring_size()
                .bits(15) // TODO replace hardcoded value
                .ring_sel()
                .set_bit()
                .treq_sel()
                .bits(rx1.dreq_value())
                .en()
                .bit(true)
                .chain_to()
                .bits(2)
        }
    });

    let dma_buf_addr = core::ptr::addr_of!(dma_buf[0]);
    defmt::info!("set dma_buf_addr {:x}", dma_buf_addr);
    let ch0_cfg: [u32; 1] = [
        //            rxf1_ptr as u32, // READ_ADDR
        //            dma_buf_addr as u32, // WRITE_ADDR
        //            dma_buf.len() as u32, // TRANS_COUNT
        ctrl.read().bits(), // CTRL_TRIG
    ];
    //defmt::info!("ch0_cfg: {:#08x}", ch0_cfg);

    unsafe {
        dma.chan_abort.write(|w| w.bits(7));
        while dma.chan_abort.read().bits() & 7 != 0 {}

        // (time, data) sm1 -> dma_buf
        dma.ch[0]
            .ch_read_addr
            .write(|w| w.bits(rx1.fifo_address() as u32));
        dma.ch[0]
            .ch_write_addr
            .write(|w| w.bits(dma_buf_addr as u32));
        //dma.ch[0].ch_read_addr.write(|w| w.bits(t_ptr as u32));
        dma.ch[0]
            .ch_trans_count
            .write(|w| w.bits(dma_buf.len() as u32));
        dma.ch[0].ch_ctrl_trig.write(|w| w.bits(ctrl.read().bits()));
        /*
        dma.ch[0].ch_ctrl_trig.write(|w| {
            w.data_size()
                .size_word()
                .incr_write()
                .bit(true)
                .treq_sel()
                .bits(rx1.dreq_value())
                .en()
                .bit(true)
                .chain_to()
                .bits(2)
        }); */
        // reconfigures and triggers channel #0 and then chains to channel #1, which doesn't need
        // reconfiguration.
        dma.ch[2]
            .ch_read_addr
            .write(|w| w.bits(core::ptr::addr_of!(ch0_cfg) as u32));
        //dma.ch[2].ch_write_addr.write(|w| w.bits(core::ptr::addr_of!(dma.ch[0].ch_read_addr) as u32));
        dma.ch[2]
            .ch_write_addr
            .write(|w| w.bits(core::ptr::addr_of!(dma.ch[0].ch_ctrl_trig) as u32));
        dma.ch[2].ch_trans_count.write(|w| w.bits(1));
        dma.ch[2].ch_ctrl_trig.write(|w| {
            w.data_size()
                .size_word()
                .incr_read()
                .bit(false)
                .incr_write()
                .bit(false)
                .treq_sel()
                .permanent()
                .en()
                .bit(true)
                .chain_to()
                .bits(1)
        });

        while !dma.ch[0].ch_ctrl_trig.read().en().bits() {
            defmt::info!("waiting for dma[0].en()");
        }
        // data sm0 -> sm1
        dma.ch[1]
            .ch_read_addr
            .write(|w| w.bits(rx0.fifo_address() as u32));
        dma.ch[1]
            .ch_write_addr
            .write(|w| w.bits(tx1.fifo_address() as u32));
        dma.ch[1]
            .ch_trans_count
            .write(|w| w.bits(dma_buf.len() as u32 / 2));
        dma.ch[1].ch_ctrl_trig.write(|w| {
            w.data_size()
                .size_word()
                .treq_sel()
                .bits(rx0.dreq_value())
                .en()
                .bit(true)
                .chain_to()
                .bits(1) // disabled
        });
        while !dma.ch[1].ch_ctrl_trig.read().en().bits() {
            defmt::info!("waiting for dma[1].en()");
        }
        /*
        while !dma.ch[2].ch_ctrl_trig.read().en().bits() {
            defmt::info!("waiting for dma[2].en()");
        }
        */
        defmt::info!("dma started");
    };
    let pio_sm1 = pio_sm1.start();
    let pio_sm0 = pio_sm0.start();
    let mut dummy = DataAccessor {
        inner: DataAccessorInner {
            dma_buf,
            read_ptr: 0,
            dma: &dma,
        },
    };
    defmt::info!("running closure");
    let result = run(&mut dummy);

    // Release PIO
    let (sm0, installed) = pio_sm0.uninit(rx0, tx0);
    pio.uninstall(installed);
    let (sm1, installed) = pio_sm1.uninit(rx1, tx1);
    pio.uninstall(installed);
    pio.free(sm0, sm1, sm2, sm3);

    // Disable DMA
    dma.ch[0].ch_ctrl_trig.write(|w| w.en().bit(false));
    dma.ch[1].ch_ctrl_trig.write(|w| w.en().bit(false));
    dma.ch[2].ch_ctrl_trig.write(|w| w.en().bit(false));
    dma.chan_abort.write(|w| unsafe { w.bits(7) });
    while dma.chan_abort.read().bits() & 7 != 0 {}
    result
}
