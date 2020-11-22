#![no_std]
#![no_main]

// Pins:
//
// Red      PB0
// Green    PB1
// Blue     PB2
// HSync    PB8
// VSync    PA5

// STM32F401CCU6

use core::num::Wrapping;

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

use stm32f4xx_hal::{
    prelude::*,
    stm32 as pac,
    nb::block,
    timer::Timer,
};
use cortex_m_rt::entry;

mod vga;
use vga::*;

const IMAGE: &'static [u8] = include_bytes!(concat!(env!("OUT_DIR"), "/image.bin"));

#[entry]
fn main() -> ! {
    rtt_init_print!();

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut led = gpioc.pc13.into_push_pull_output();
    led.set_high().ok();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .use_hse(25.mhz())
        .sysclk(75.mhz()).freeze();

    rprintln!("hclk {} pclk1 {} pclk2 {} sysclk {}",
        clocks.hclk().0,
        clocks.pclk1().0,
        clocks.pclk2().0,
        clocks.sysclk().0);

    let mut timer = Timer::syst(cp.SYST, 10.hz(), clocks);

    let mut vga = VGA::new(
        dp.TIM1,
        (gpiob.pb0, gpiob.pb1, gpiob.pb2),
        gpiob.pb3,
        gpioa.pa5);

    vga.enable();

    vga.draw(IMAGE);

    let mut rng = XorShift::new();

    loop {
        /*led.set_low().ok();
        block!(timer.wait()).unwrap();
        block!(timer.wait()).unwrap();
        led.set_high().ok();
        block!(timer.wait()).unwrap();*/

        vga.wait_vblank();

        for _ in 0..100 {
            vga.plot(rng.next_u32() % WIDTH, rng.next_u32() % HEIGHT, 0);
        }
    }
}

struct XorShift {
    x: Wrapping<u32>,
    y: Wrapping<u32>,
    z: Wrapping<u32>,
    w: Wrapping<u32>,
}

impl XorShift {
    pub fn new() -> Self {
        return Self {
            x: Wrapping(0x469eb0a3),
            y: Wrapping(0x99d6f0c0),
            z: Wrapping(0x0436579b),
            w: Wrapping(0xd46197ff),
        }
    }

    pub fn next_u32(&mut self) -> u32 {
        let x = self.x;
        let t = x ^ (x << 11);
        self.x = self.y;
        self.y = self.z;
        self.z = self.w;
        let w_ = self.w;
        self.w = w_ ^ (w_ >> 19) ^ (t ^ (t >> 8));
        self.w.0
    }
}