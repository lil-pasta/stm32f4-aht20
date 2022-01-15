#![allow(dead_code)]
#![no_main]
#![no_std]

use panic_halt as _;

use aht20::drivers::aht20::Aht20;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f4xx_hal::{delay::Delay, i2c::I2c, pac, prelude::*};

#[entry]
fn main() -> ! {
    // let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    let mut t2_delay = Delay::tim2(dp.TIM2, &clocks);
    let mut t5_delay = Delay::tim5(dp.TIM5, &clocks);
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let led = gpioc.pc13.into_push_pull_output();
    let scl = gpiob.pb6.into_alternate_open_drain();
    let sck = gpiob.pb7.into_alternate_open_drain();

    let mut i2c1 = I2c::new(dp.I2C1, (scl, sck), 100.khz(), clocks);
    // let bus = shared_bus::BusManagerSimple::new(i2c1);
    let mut aht20 = Aht20::new(i2c1, t2_delay).unwrap();
    loop {
        let temp_hum = aht20.get_env().unwrap();
        hprintln!(
            "temp_c: {}\nrel_hum: {}\n",
            temp_hum.0.calculate_c(),
            temp_hum.1.calculate_rh()
        )
        .unwrap();
        t5_delay.delay_ms(1000u16)
    }
}
