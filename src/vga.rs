use core::sync::atomic::{AtomicU32, Ordering::SeqCst};
use cortex_m::peripheral::NVIC;
use stm32f4xx_hal::{
    gpio::{
        Floating,
        Input,
        gpioa::PA5,
        gpiob::{PB0, PB1, PB2, PB3},
    },
    stm32::{
        interrupt,
        Interrupt,
        DMA2,
        GPIOA,
        GPIOB,
        RCC,
        TIM1,
    }
};

use rtt_target::rprintln;

pub struct VGA {
    timer: TIM1,
}

// 640 x 480 @ 60Hz with divisor
mod timing {
    // PIXEL_CLOCK_HZ = 25_175_000 / HDIV;

    // pixels
    pub const HDIV: u32 = 3;
    pub const HVIDEO: u32 = 640 / HDIV;
    pub const HFRONT: u32 = 16 / HDIV;
    pub const HSYNC: u32 = 96 / HDIV;
    pub const HBACK: u32 = 48 / HDIV;

    pub const HTOTAL: u32 = HVIDEO + HFRONT + HSYNC + HBACK;

    // lines
    pub const VDIV: u32 = 3;
    pub const VVIDEO: u32 = 480;
    pub const VFRONT: u32 = 11;
    pub const VSYNC: u32 = 2;
    pub const VBACK: u32 = 31;

    pub const VTOTAL: u32 = VVIDEO + VFRONT + VSYNC + VBACK;
}

pub const WIDTH: u32 = timing::HVIDEO;
pub const HEIGHT: u32 = timing::VVIDEO / timing::VDIV;

const LINE_INIT: u32 = (0b1111 << 16) | 0b1000;
const BUFLEN: usize = timing::HTOTAL as usize;

static mut LINE_BUF: [[u32; BUFLEN]; 2] = [[0; BUFLEN], [0; BUFLEN]];
static mut FRAME_BUF: [u8; ((WIDTH / 2) * HEIGHT) as usize] = [0; ((WIDTH / 2) * HEIGHT) as usize];
static LINE: AtomicU32 = AtomicU32::new(0);

impl VGA {
    pub fn new(
        timer: TIM1,
        color_pins: (PB0<Input<Floating>>, PB1<Input<Floating>>, PB2<Input<Floating>>),
        hsync: PB3<Input<Floating>>,
        vsync: PA5<Input<Floating>>) -> Self
    {
        unsafe {
            let rcc = &(*RCC::ptr());
            rcc.ahb1enr.modify(|_, w| w.dma2en().enabled());
            rcc.ahb1rstr.modify(|_, w| w.dma2rst().reset());
            rcc.ahb1rstr.modify(|_, w| w.dma2rst().clear_bit());

            rcc.apb2enr.modify(|_, w| w.tim1en().enabled());
            rcc.apb2rstr.modify(|_, w| w.tim1rst().reset());
            rcc.apb2rstr.modify(|_, w| w.tim1rst().clear_bit());
        }

        color_pins.0.into_push_pull_output();
        color_pins.1.into_push_pull_output();
        color_pins.2.into_push_pull_output();
        hsync.into_push_pull_output();
        vsync.into_push_pull_output();

        unsafe {
            let port = &*GPIOB::ptr();

            port.ospeedr.modify(|_, w| {
                w.ospeedr0().very_high_speed();
                w.ospeedr1().very_high_speed();
                w.ospeedr2().very_high_speed();
                w.ospeedr3().very_high_speed();
                w
            });
        }

        VGA {
            timer,
        }
    }

    pub fn enable(&mut self) {
        unsafe {
            // Init buffers

            for buf in LINE_BUF.iter_mut() {
                let p = buf.as_mut_ptr();
                for x in 0..timing::HTOTAL {
                    let mut v: u32 = LINE_INIT;

                    if x >= timing::HVIDEO + timing::HFRONT && x < timing::HTOTAL - timing::HBACK {
                        v &= !0b1000;
                    }

                    p.add(x as usize).write_volatile(v);
                }
            }

            NVIC::unmask(Interrupt::TIM1_UP_TIM10);

            NVIC::unmask(Interrupt::DMA2_STREAM5);

            // DMA

            let dma = &*DMA2::ptr();
            let stream = &dma.st[5];

            // stream 5, channel 6
            stream.cr.write(|w| {
                w.tcie().enabled();
                w.dir().memory_to_peripheral();
                w.circ().enabled();
                w.minc().incremented();
                w.psize().bits32();
                w.msize().bits32();
                w.pl().very_high();
                w.dbm().enabled();
                w.chsel().bits(6);
                w
            });

            stream.ndtr.write(|w| w.bits(LINE_BUF[0].len() as u32));

            let port = (&(&*GPIOB::ptr()).bsrr) as *const _ as *const u32;
            stream.par.write(|w| w.bits(port as u32));

            stream.m0ar.write(|w| w.bits(LINE_BUF[0].as_ptr() as u32));
            stream.m1ar.write(|w| w.bits(LINE_BUF[0].as_ptr() as u32));

            stream.cr.modify(|_, w| w.en().enabled());

            rprintln!("pollo");

            // Timer

            let timer = &self.timer;

            timer.arr.write(|w| w.arr().bits(8));

            timer.dier.write(|w| w.ude().enabled());

            timer.cr1.modify(|_, w| {
                w.urs().counter_only();
                w
            });

            timer.cr1.modify(|_, w| w.cen().enabled());

            timer.egr.write(|w| w.ug().set_bit());
        }
    }

    pub fn plot(&mut self, x: u32, y: u32, color: u8) {
        if x >= WIDTH || y >= HEIGHT {
            return;
        }

        unsafe {
            let p = FRAME_BUF.as_mut_ptr().add((y * (WIDTH / 2) + x / 2) as usize);
            let v = p.read_volatile();

            p.write_volatile(if x & 1 == 0 {
                (v & !0b1111) | color
            } else {
                (v & !0b11110000) | (color << 4)
            });
        }
    }

    pub fn draw(&mut self, image: &[u8]) {
        unsafe {
            let mut fp = FRAME_BUF.as_mut_ptr();

            for b in image.iter().take(FRAME_BUF.len()) {
                fp.write_volatile(*b);
                fp = fp.add(1);
            }
        }
    }

    pub fn wait_vblank(&self) {
        while LINE.load(SeqCst) >= timing::VVIDEO {
            core::sync::atomic::spin_loop_hint();
        }

        while LINE.load(SeqCst) < timing::VVIDEO {
            core::sync::atomic::spin_loop_hint();
        }
    }
}

#[interrupt]
unsafe fn DMA2_STREAM5() {
    use timing::VDIV;

    let dma = &*DMA2::ptr();
    let stream = &dma.st[5];

    let ct = stream.cr.read().ct().bits() as usize;
    dma.hifcr.write(|w| w.ctcif5().set_bit());

    let line = match LINE.fetch_add(1, SeqCst) {
        timing::VTOTAL => {
            LINE.store(0, SeqCst);
            0
        }
        l => l,
    };

    let port = &*GPIOA::ptr();

    if line == timing::VTOTAL - timing::VBACK - timing::VSYNC {
        port.bsrr.write(|w| w.bits(1 << (5 + 16)));
    } else if line == timing::VTOTAL - timing::VBACK {
        port.bsrr.write(|w| w.bits(1 << 5));
    }

    let phase = line % (VDIV * 2);

    let write_buf_index = if phase < VDIV { 1 } else { 0 };
    let read_buf_index = 1 - write_buf_index;

    let next_buf = LINE_BUF[
        if phase == VDIV - 1 || phase == VDIV * 2 - 1 {
            write_buf_index
        } else {
            read_buf_index
        }].as_ptr() as u32;

    if ct == 0 {
        stream.m1ar.write(|w| w.bits(next_buf));
    } else {
        stream.m0ar.write(|w| w.bits(next_buf));
    }

    let bp = LINE_BUF[write_buf_index].as_mut_ptr();

    if line < timing::VVIDEO {
        if phase == 0 || phase == VDIV {
            render_line(bp, line / VDIV);
        }
    } else {
        for i in 0..WIDTH {
            bp.add(i as usize).write_volatile(LINE_INIT);
        }
    }
}

fn render_line(mut bp: *mut u32, y: u32) {
    unsafe {
        let mut fp = FRAME_BUF.as_ptr().add((y * (WIDTH / 2)) as usize);

        for _ in 0..WIDTH / 2 {
            let byte = fp.read_volatile();

            bp.write_volatile(LINE_INIT | ((byte & 0b111) as u32));
            bp = bp.add(1);
            bp.write_volatile(LINE_INIT | (((byte >> 4) & 0b111) as u32));
            bp = bp.add(1);

            fp = fp.add(1);
        }
    }
}
