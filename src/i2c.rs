use embedded_hal_async::i2c::I2c as I2cTrait;

use crate::{Bus, Channel, Registers};

pub struct I2c<I2C> {
    address: u8,
    i2c: I2C,
}

impl<I2C> I2c<I2C>
where
    I2C: I2cTrait,
{
    pub fn new(address: u8, i2c: I2C) -> Self {
        Self { address, i2c }
    }
}

impl<I2C> Bus for I2c<I2C>
where
    I2C: I2cTrait,
{
    type Error = I2C::Error;

    async fn read_register(
        &mut self,
        channel: Channel,
        register: Registers,
    ) -> Result<u8, Self::Error> {
        let register_address = (register as u8) | (channel as u8);
        let mut buffer = [0u8; 1];
        self.i2c
            .write_read(self.address, &[register_address], &mut buffer)
            .await?;
        Ok(buffer[0])
    }

    async fn write_register(
        &mut self,
        channel: Channel,
        register: Registers,
        data: u8,
    ) -> Result<(), Self::Error> {
        let register_address = (register as u8) | (channel as u8);
        let buffer = [register_address, data];
        self.i2c.write(self.address, &buffer).await
    }
}
