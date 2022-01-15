#![allow(dead_code)]

use bitflags::bitflags;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

use cortex_m_semihosting::hprintln;

const I2C_ADDRESS: u8 = 0x38;

#[derive(Debug)]
pub enum Error<E> {
    BusError(E),
    CalibrationError,
}

struct Commands;
impl Commands {
    // documented calibration command 0xbe is not right
    // found 0xe1 used in adafruit official driver...
    const CALIBRATE: [u8; 3] = [0xE1, 0x08, 0x00];
    const MEASURE: [u8; 3] = [0xAC, 0x33, 0x00];
    const SFT_RST: [u8; 1] = [0xBA];
    const GET_STAT: [u8; 1] = [0x00];
}

bitflags! {
    pub struct Status: u8 {
        const BUSY = (1 << 7);
        const CAL_ENABLE = (1 << 3);
        const _ZERO = (1 << 0);
        const _ONE = (1 << 1);
        const _TWO = (1 << 2);
        const _FIVE = (1 << 4);
        const _SIX = ((1 << 6) | (1 << 5));
    }
}

pub struct Aht20<I, D> {
    i2c: I,
    delay: D,
}

impl<I, E, D> Aht20<I, D>
where
    I: Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u16>,
{
    pub fn new(i2c: I, delay: D) -> Result<Self, Error<E>> {
        let mut sensor = Aht20 { i2c, delay };
        sensor.reset()?;
        sensor.cal()?;
        Ok(sensor)
    }

    fn reset(&mut self) -> Result<(), Error<E>> {
        self.write(&Commands::SFT_RST)?;
        self.delay.delay_ms(20u16);
        Ok(())
    }

    fn cal(&mut self) -> Result<(), Error<E>> {
        self.delay.delay_ms(40u16);
        self.write(&Commands::CALIBRATE)?;

        while self.get_status()?.contains(Status::BUSY) {
            self.delay.delay_ms(10u16);
        }

        if !self.get_status()?.contains(Status::CAL_ENABLE) {
            return Err(Error::CalibrationError);
        }

        Ok(())
    }

    pub fn get_env(&mut self) -> Result<(Temperature, Humidity), Error<E>> {
        let temp_hum = self.read(&Commands::GET_STAT)?;
        let temp = Temperature { raw: temp_hum.0 };
        let hum = Humidity { raw: temp_hum.1 };
        Ok((temp, hum))
    }

    // convenience functions if only one reading is desired.
    pub fn get_temp(&mut self) -> Result<Temperature, Error<E>> {
        Ok(self.get_env()?.0)
    }

    pub fn get_hum(&mut self) -> Result<Humidity, Error<E>> {
        Ok(self.get_env()?.1)
    }

    // rewrite the read function so you can reuse it here instead of a custom
    // write_read call to handle the status
    fn get_status(&mut self) -> Result<Status, Error<E>> {
        let buf: &mut [u8; 1] = &mut [0x00];
        self.i2c
            .write_read(I2C_ADDRESS, &[0u8], buf)
            .map_err(|e| Error::BusError(e))?;
        Ok(Status { bits: buf[0] })
    }

    fn write(&mut self, command: &[u8]) -> Result<(), Error<E>> {
        self.i2c
            .write(I2C_ADDRESS, command)
            .map_err(Error::BusError)?;
        Ok(())
    }

    fn read(&mut self, command: &[u8]) -> Result<(u32, u32), Error<E>> {
        let buf: &mut [u8; 7] = &mut [0x00; 7];

        // tell the device to measure
        self.write(&Commands::MEASURE)?;

        while self.get_status()?.contains(Status::BUSY) {
            hprintln!("busy").unwrap();
            self.delay.delay_ms(10u16);
        }

        // once the busy bit isnt on you can read the result
        self.i2c
            .write_read(I2C_ADDRESS, command, buf)
            .map_err(|e| Error::BusError(e))?;

        let s = Status { bits: buf[0] };
        if !s.contains(Status::CAL_ENABLE) {
            return Err(Error::CalibrationError);
        }

        let hum_raw = (buf[1] as u32) << 12 | (buf[2] as u32) << 4 | (buf[3] as u32) >> 4;
        let temp_raw = ((buf[3] as u32) & 0xf) << 16 | (buf[4] as u32) << 8 | (buf[5] as u32);
        Ok((temp_raw, hum_raw))
    }
}

pub struct Temperature {
    raw: u32,
}

impl Temperature {
    pub fn calculate_c(&self) -> f32 {
        (self.raw as f32) / ((1 << 20) as f32) * 200.0 - 50.0
    }

    pub fn calculate_f(&self) -> f32 {
        self.calculate_c() * (9.0 / 5.0) + 32.0
    }
}

pub struct Humidity {
    raw: u32,
}

impl Humidity {
    pub fn calculate_rh(&self) -> f32 {
        (self.raw as f32) / ((1 << 20) as f32) * 100.0
    }
}
