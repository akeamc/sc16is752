use embedded_hal_async::spi::SpiDevice;

use crate::{Bus, Channel, Registers};

pub struct Spi<SPI> {
    spi: SPI,
}

impl<SPI> Spi<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

impl<SPI> Bus for Spi<SPI>
where
    SPI: SpiDevice,
{
    type Error = SPI::Error;

    async fn read_register(
        &mut self,
        channel: Channel,
        register: Registers,
    ) -> Result<u8, Self::Error> {
        let register_address = (register as u8) | (channel as u8) | 0x80;
        let mut buffer = [register_address, 0x00];
        self.spi.transfer_in_place(&mut buffer).await?;
        Ok(buffer[1])
    }

    async fn write_register(
        &mut self,
        channel: Channel,
        register: Registers,
        data: u8,
    ) -> Result<(), Self::Error> {
        let register_address = (register as u8) | (channel as u8);
        let buffer = [register_address, data];
        self.spi.write(&buffer).await
    }
}
