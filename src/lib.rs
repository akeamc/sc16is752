//!
//! # Example
//!
//! ```ignore
//! use sc16is752::*;
//!
//! async fn example() -> Result<(), Box<dyn std::error::Error>> {
//!     // Initialize with I2C
//!     let mut device = SC16IS752::new(SC16IS752i2c::new(0x48, i2c), 14_745_600);
//!     // Or for SPI:
//!     // let mut device = SC16IS752::new(SC16IS752spi::new(spi_dev), 14_745_600);
//!
//!     device.initialise_uart(Channel::A, UartConfig::default().baudrate(9600)).await?;
//!     device.gpio_set_pin_mode(GPIO::GPIO0, PinMode::Output).await?;
//!     device.flush(Channel::A).await?;
//!
//!     // Write some data
//!     device.write(Channel::A, b"Hello, World!").await?;
//!
//!     // Read available data
//!     let mut buffer = [0u8; 64];
//!     let bytes_read = device.read(Channel::A, &mut buffer).await?;
//!
//!     // GPIO operations
//!     device.gpio_set_pin_state(GPIO::GPIO0, PinState::High).await?;
//!     let pin_state = device.gpio_get_pin_state(GPIO::GPIO0).await?;
//!
//!     Ok(())
//! }
//! ```

#![no_std]
#![allow(async_fn_in_trait)]

use embedded_hal_async::i2c::I2c;
use embedded_hal_async::spi::SpiDevice;
use embedded_io_async::{ErrorType, Read as AsyncRead, Write as AsyncWrite};

/// UARTs Channel A (TXA/RXA) and Channel B (TXB/RXB)
#[derive(Debug, Copy, Clone)]
#[repr(u8)]
pub enum Channel {
    A = 0x00,
    B = 0x01,
}
impl core::fmt::Display for Channel {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// SC16IS752 internal register address
#[derive(Debug, Copy, Clone)]
pub enum Registers {
    RhrThr = 0x00 << 3,
    IER = 0x01 << 3,
    FcrIir = 0x02 << 3,
    LCR = 0x03 << 3,
    MCR = 0x04 << 3,
    LSR = 0x05 << 3,
    MsrTcr = 0x06 << 3,
    SprTlr = 0x07 << 3,
    TXLVL = 0x08 << 3,
    RXLVL = 0x09 << 3,
    IODir = 0x0A << 3,
    IOState = 0x0B << 3,
    IOIntEna = 0x0C << 3,
    IOControl = 0x0E << 3,
    EFCR = 0x0F << 3,
}

/// General Purpose IO pins
///
#[derive(Debug, Copy, Clone)]
pub enum GPIO {
    GPIO0 = 0x01,
    GPIO1 = 0x02,
    GPIO2 = 0x04,
    GPIO3 = 0x08,
    GPIO4 = 0x10,
    GPIO5 = 0x20,
    GPIO6 = 0x40,
    GPIO7 = 0x80,
}
impl core::fmt::Display for GPIO {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// GPIO pin mode
#[derive(Debug, Copy, Clone)]
pub enum PinMode {
    Input = 0,
    Output = 1,
}
impl core::fmt::Display for PinMode {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// GPIO pin state
#[derive(Debug, Copy, Clone)]
pub enum PinState {
    Low = 0,
    High = 1,
}
impl core::fmt::Display for PinState {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

/// Interrupt Event Test
#[derive(Debug, Copy, Clone)]
pub enum InterruptEventTest {
    ReceiveLineStatusError = 0x06,
    ReceiveTimeoutInterrupt = 0x0C,
    RhrInterrupt = 0x04,
    ThrInterrupt = 0x02,
    ModemInterrupt = 0x00,
    InputPinChangeState = 0x30,
    ReceiveXoff = 0x10,
    CtsRtsChange = 0x20,
    Unknown = 0xFF,
}

/// Enhanced Features Control Register (EFCR) bits
#[derive(Debug, Copy, Clone)]
pub enum FeaturesRegister {
    /// Enable IrDA mode
    /// 0 = normal mode (default)
    /// 1 = IrDA mode
    IrDaFast = 0x01,
    /// Enable automatic RS-485 support
    /// 0 = disable (default)
    /// 1 = enable
    AutoRs485RTSOutputInversion = 0x20,
    /// Enable automatic RS-485 support
    /// 0 = disable (default)
    /// 1 = enable
    AutoRs485DirectionControl = 0x10,
    /// Disable transmitter
    /// 0 = enable (default)
    /// 1 = disable
    TxDisable = 0x04,
    /// Disable receiver
    /// 0 = enable (default)
    /// 1 = disable
    RxDisable = 0x02,
    /// Enable 9-bit or Multidrop mode (RS-485)
    Multidrop = 0x08,
}

/// Parity configuration
#[derive(Debug, Copy, Clone)]
pub enum Parity {
    NoParity = 0x00,
    Odd = 0x08,
    Even = 0x18,
    ForcedParity1 = 0x28, // force parity to 1
    ForcedParity0 = 0x38, // force parity to 0
}

/// UART configuration
#[derive(Debug, Copy, Clone)]
pub struct UartConfig {
    pub baud: u32,
    pub word_length: u8,
    pub parity: Parity,
    pub stop_bit: u8,
}

impl UartConfig {
    /// Create a new UART configuration
    ///
    /// # Arguments
    /// * `baud` - Baud rate (e.g., 9600, 115200)
    /// * `word_length` - Data bits (5-8)
    /// * `parity` - Parity setting
    /// * `stop_bit` - Stop bits (1-2)
    pub fn new(baud: u32, word_length: u8, parity: Parity, stop_bit: u8) -> Self {
        Self {
            baud,
            word_length,
            parity,
            stop_bit,
        }
    }

    pub fn baudrate(mut self, baud: u32) -> Self {
        self.baud = baud;
        self
    }
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baud: 9600,
            word_length: 8,
            parity: Parity::NoParity,
            stop_bit: 1,
        }
    }
}

/// SC16IS752 error types
#[derive(Debug)]
pub enum SC16IS752Error<E> {
    Bus(E),
}

impl<E> core::fmt::Display for SC16IS752Error<E>
where
    E: core::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            SC16IS752Error::Bus(err) => write!(f, "Bus error: {:?}", err),
        }
    }
}

impl<E> core::error::Error for SC16IS752Error<E> where E: core::fmt::Debug {}

impl<E> embedded_io_async::Error for SC16IS752Error<E>
where
    E: core::fmt::Debug,
{
    fn kind(&self) -> embedded_io_async::ErrorKind {
        embedded_io_async::ErrorKind::Other
    }
}

impl<E> From<E> for SC16IS752Error<E> {
    fn from(error: E) -> Self {
        SC16IS752Error::Bus(error)
    }
}

pub trait Bus {
    type Error: core::fmt::Debug;

    async fn write_register(
        &mut self,
        channel: Channel,
        register: Registers,
        data: u8,
    ) -> Result<(), Self::Error>;

    async fn read_register(
        &mut self,
        channel: Channel,
        register: Registers,
    ) -> Result<u8, Self::Error>;
}

pub struct SC16IS752i2c<I2C> {
    address: u8,
    i2c: I2C,
}

impl<I2C> SC16IS752i2c<I2C>
where
    I2C: I2c,
{
    pub fn new(address: u8, i2c: I2C) -> Self {
        Self { address, i2c }
    }
}

impl<I2C> Bus for SC16IS752i2c<I2C>
where
    I2C: I2c,
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

pub struct SC16IS752spi<SPI> {
    spi: SPI,
}

impl<SPI> SC16IS752spi<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }
}

impl<SPI> Bus for SC16IS752spi<SPI>
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

pub struct SC16IS752<BUS>
where
    BUS: Bus,
{
    bus: BUS,
    xtal_freq: u32,
}

pub struct ChannelWrapper<'a, BUS>
where
    BUS: Bus,
{
    device: &'a mut SC16IS752<BUS>,
    channel: Channel,
}

impl<'a, BUS> ChannelWrapper<'a, BUS>
where
    BUS: Bus,
{
    pub fn new(device: &'a mut SC16IS752<BUS>, channel: Channel) -> Self {
        Self { device, channel }
    }
}

impl<BUS> ErrorType for ChannelWrapper<'_, BUS>
where
    BUS: Bus,
{
    type Error = SC16IS752Error<BUS::Error>;
}

impl<BUS> AsyncWrite for ChannelWrapper<'_, BUS>
where
    BUS: Bus,
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.device.write(self.channel, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.device.flush(self.channel).await
    }
}

impl<BUS> AsyncRead for ChannelWrapper<'_, BUS>
where
    BUS: Bus,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.device.read(self.channel, buf).await
    }
}

pub struct OwnedChannelWrapper<BUS>
where
    BUS: Bus,
{
    device: SC16IS752<BUS>,
    channel: Channel,
}

impl<BUS> OwnedChannelWrapper<BUS>
where
    BUS: Bus,
{
    pub fn new(device: SC16IS752<BUS>, channel: Channel) -> Self {
        Self { device, channel }
    }
}

impl<BUS> ErrorType for OwnedChannelWrapper<BUS>
where
    BUS: Bus,
{
    type Error = SC16IS752Error<BUS::Error>;
}

impl<BUS> AsyncWrite for OwnedChannelWrapper<BUS>
where
    BUS: Bus,
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.device.write(self.channel, buf).await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        self.device.flush(self.channel).await
    }
}

impl<BUS> AsyncRead for OwnedChannelWrapper<BUS>
where
    BUS: Bus,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.device.read(self.channel, buf).await
    }
}

impl<BUS> SC16IS752<BUS>
where
    BUS: Bus,
{
    pub fn new(bus: BUS, xtal_freq: u32) -> Self {
        Self { bus, xtal_freq }
    }

    pub fn get_channel(&mut self, channel: Channel) -> ChannelWrapper<'_, BUS> {
        ChannelWrapper::new(self, channel)
    }

    pub fn into_channel(self, channel: Channel) -> OwnedChannelWrapper<BUS> {
        OwnedChannelWrapper::new(self, channel)
    }

    pub async fn initialise_uart(
        &mut self,
        channel: Channel,
        config: UartConfig,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.set_baudrate(channel, config.baud).await?;
        self.set_line(channel, config.word_length, config.parity, config.stop_bit)
            .await?;
        Ok(())
    }

    async fn read_register(
        &mut self,
        channel: Channel,
        register: Registers,
    ) -> Result<u8, SC16IS752Error<BUS::Error>> {
        Ok(self.bus.read_register(channel, register).await?)
    }

    async fn write_register(
        &mut self,
        channel: Channel,
        register: Registers,
        data: u8,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.bus.write_register(channel, register, data).await?;
        Ok(())
    }

    async fn set_baudrate(
        &mut self,
        channel: Channel,
        baud: u32,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let divisor = self.xtal_freq / baud / 16;
        let lcr = self.read_register(channel, Registers::LCR).await?;
        self.write_register(channel, Registers::LCR, lcr | 0x80)
            .await?;
        self.write_register(channel, Registers::RhrThr, (divisor & 0xff) as u8)
            .await?;
        self.write_register(channel, Registers::IER, ((divisor >> 8) & 0xff) as u8)
            .await?;
        self.write_register(channel, Registers::LCR, lcr & 0x7F)
            .await?;
        Ok(())
    }

    async fn set_line(
        &mut self,
        channel: Channel,
        word_length: u8,
        parity: Parity,
        stop_bit: u8,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut lcr = 0u8;

        // Set word length
        match word_length {
            5 => lcr |= 0x00,
            6 => lcr |= 0x01,
            7 => lcr |= 0x02,
            8 => lcr |= 0x03,
            _ => {
                return Err(SC16IS752Error::Bus(
                    self.bus
                        .read_register(channel, Registers::LCR)
                        .await
                        .unwrap_err(),
                ))
            }
        }

        // Set parity
        lcr |= parity as u8;

        // Set stop bits
        if stop_bit == 2 {
            lcr |= 0x04;
        }

        self.write_register(channel, Registers::LCR, lcr).await
    }

    pub async fn fifo_available_data(
        &mut self,
        channel: Channel,
    ) -> Result<u8, SC16IS752Error<BUS::Error>> {
        self.read_register(channel, Registers::RXLVL).await
    }

    pub async fn fifo_available_space(
        &mut self,
        channel: Channel,
    ) -> Result<u8, SC16IS752Error<BUS::Error>> {
        let space = 64 - self.read_register(channel, Registers::TXLVL).await?;
        Ok(space)
    }

    async fn write_byte(
        &mut self,
        channel: Channel,
        byte: u8,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.write_register(channel, Registers::RhrThr, byte).await
    }

    pub async fn write(
        &mut self,
        channel: Channel,
        bytes: &[u8],
    ) -> Result<usize, SC16IS752Error<BUS::Error>> {
        for byte in bytes {
            self.write_byte(channel, *byte).await?;
        }
        Ok(bytes.len())
    }

    async fn read_byte(
        &mut self,
        channel: Channel,
    ) -> Result<Option<u8>, SC16IS752Error<BUS::Error>> {
        let available = self.fifo_available_data(channel).await?;
        if available > 0 {
            Ok(Some(self.read_register(channel, Registers::RhrThr).await?))
        } else {
            Ok(None)
        }
    }

    pub async fn read(
        &mut self,
        channel: Channel,
        buffer: &mut [u8],
    ) -> Result<usize, SC16IS752Error<BUS::Error>> {
        let mut count = 0;
        for item in buffer.iter_mut() {
            if let Some(byte) = self.read_byte(channel).await? {
                *item = byte;
                count += 1;
            } else {
                break;
            }
        }
        Ok(count)
    }

    pub async fn flush(&mut self, channel: Channel) -> Result<(), SC16IS752Error<BUS::Error>> {
        loop {
            let txlvl = self.read_register(channel, Registers::TXLVL).await?;
            if txlvl == 0 {
                break;
            }
        }
        Ok(())
    }

    pub async fn gpio_set_pin_mode(
        &mut self,
        pin: GPIO,
        mode: PinMode,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut current_state = self.read_register(Channel::A, Registers::IODir).await?;
        match mode {
            PinMode::Input => current_state &= !(pin as u8),
            PinMode::Output => current_state |= pin as u8,
        }
        self.write_register(Channel::A, Registers::IODir, current_state)
            .await
    }

    pub async fn gpio_set_pin_state(
        &mut self,
        pin: GPIO,
        state: PinState,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut current_state = self.read_register(Channel::A, Registers::IOState).await?;
        match state {
            PinState::Low => current_state &= !(pin as u8),
            PinState::High => current_state |= pin as u8,
        }
        self.write_register(Channel::A, Registers::IOState, current_state)
            .await
    }

    pub async fn gpio_get_pin_state(
        &mut self,
        pin: GPIO,
    ) -> Result<PinState, SC16IS752Error<BUS::Error>> {
        let state = self.read_register(Channel::A, Registers::IOState).await?;
        if (state & pin as u8) != 0 {
            Ok(PinState::High)
        } else {
            Ok(PinState::Low)
        }
    }

    pub async fn gpio_get_port_state(&mut self) -> Result<u8, SC16IS752Error<BUS::Error>> {
        self.read_register(Channel::A, Registers::IOState).await
    }

    pub async fn gpio_set_port_mode(&mut self, mode: u8) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.write_register(Channel::A, Registers::IODir, mode)
            .await
    }

    pub async fn gpio_set_port_state(
        &mut self,
        state: u8,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.write_register(Channel::A, Registers::IOState, state)
            .await
    }

    pub async fn set_pin_interrupt(
        &mut self,
        pin: GPIO,
        enable: bool,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut current_state = self.read_register(Channel::A, Registers::IOIntEna).await?;
        match enable {
            false => current_state &= !(pin as u8),
            true => current_state |= pin as u8,
        }
        self.write_register(Channel::A, Registers::IOIntEna, current_state)
            .await
    }

    pub async fn reset_device(&mut self) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.write_register(Channel::A, Registers::IOControl, 0x08)
            .await
    }

    pub async fn modem_pin(
        &mut self,
        channel: Channel,
        pin: GPIO,
        state: PinState,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut current_state = self.read_register(channel, Registers::MCR).await?;
        match state {
            PinState::Low => current_state &= !(pin as u8),
            PinState::High => current_state |= pin as u8,
        }
        self.write_register(channel, Registers::MCR, current_state)
            .await
    }

    pub async fn gpio_latch(
        &mut self,
        channel: Channel,
        pin: GPIO,
        state: PinState,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut current_state = self.read_register(channel, Registers::SprTlr).await?;
        match state {
            PinState::Low => current_state &= !(pin as u8),
            PinState::High => current_state |= pin as u8,
        }
        self.write_register(channel, Registers::SprTlr, current_state)
            .await
    }

    pub async fn interrupt_control(&mut self, mode: u8) -> Result<(), SC16IS752Error<BUS::Error>> {
        self.write_register(Channel::A, Registers::IOControl, mode)
            .await
    }

    pub async fn interrupt_pending_test(&mut self) -> Result<bool, SC16IS752Error<BUS::Error>> {
        let result = self.read_register(Channel::A, Registers::IOIntEna).await?;
        Ok((result & 0x01) == 0x00)
    }

    pub async fn isr(
        &mut self,
        channel: Channel,
    ) -> Result<InterruptEventTest, SC16IS752Error<BUS::Error>> {
        let result = self.read_register(channel, Registers::FcrIir).await? & 0x3F;

        match result {
            0x06 => Ok(InterruptEventTest::ReceiveLineStatusError),
            0x0C => Ok(InterruptEventTest::ReceiveTimeoutInterrupt),
            0x04 => Ok(InterruptEventTest::RhrInterrupt),
            0x02 => Ok(InterruptEventTest::ThrInterrupt),
            0x00 => Ok(InterruptEventTest::ModemInterrupt),
            0x30 => Ok(InterruptEventTest::InputPinChangeState),
            0x10 => Ok(InterruptEventTest::ReceiveXoff),
            0x20 => Ok(InterruptEventTest::CtsRtsChange),
            _ => Ok(InterruptEventTest::Unknown),
        }
    }

    pub async fn fifo_enable(
        &mut self,
        channel: Channel,
        fifo_enable: bool,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut fcr = 0x00;
        if fifo_enable {
            fcr |= 0x01;
        }
        self.write_register(channel, Registers::FcrIir, fcr).await
    }

    pub async fn fifo_reset(
        &mut self,
        channel: Channel,
        reset_tx: bool,
        reset_rx: bool,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut fcr = 0x01; // Enable FIFO
        if reset_tx {
            fcr |= 0x04;
        }
        if reset_rx {
            fcr |= 0x02;
        }
        self.write_register(channel, Registers::FcrIir, fcr).await
    }

    pub async fn fifo_set_trigger_level(
        &mut self,
        channel: Channel,
        rx_trigger: u8,
        tx_trigger: u8,
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut fcr = 0x01; // Enable FIFO

        // Set RX trigger level (bits 7:6)
        match rx_trigger {
            8 => fcr |= 0x00,
            16 => fcr |= 0x40,
            56 => fcr |= 0x80,
            60 => fcr |= 0xC0,
            _ => {
                return Err(SC16IS752Error::Bus(
                    self.bus
                        .read_register(channel, Registers::FcrIir)
                        .await
                        .unwrap_err(),
                ))
            }
        }

        // Set TX trigger level (bits 5:4)
        match tx_trigger {
            8 => fcr |= 0x00,
            16 => fcr |= 0x10,
            32 => fcr |= 0x20,
            56 => fcr |= 0x30,
            _ => {
                return Err(SC16IS752Error::Bus(
                    self.bus
                        .read_register(channel, Registers::FcrIir)
                        .await
                        .unwrap_err(),
                ))
            }
        }

        self.write_register(channel, Registers::FcrIir, fcr).await
    }

    pub async fn enable_features(
        &mut self,
        channel: Channel,
        features: &[FeaturesRegister],
    ) -> Result<(), SC16IS752Error<BUS::Error>> {
        let mut efcr = 0u8;
        for feature in features {
            efcr |= *feature as u8;
        }

        // Enable enhanced features
        let lcr = self.read_register(channel, Registers::LCR).await?;
        self.write_register(channel, Registers::LCR, 0xBF).await?; // Access EFR
        let efr = self.read_register(channel, Registers::FcrIir).await?;
        self.write_register(channel, Registers::FcrIir, efr | 0x10)
            .await?; // Enable enhanced functions
        self.write_register(channel, Registers::LCR, lcr).await?; // Restore LCR

        // Set EFCR
        self.write_register(channel, Registers::EFCR, efcr).await
    }

    pub async fn ping(&mut self) -> Result<bool, SC16IS752Error<BUS::Error>> {
        const TEST_CHAR: u8 = 0x55;

        // Save original SPR value
        let original_spr_a = self.read_register(Channel::A, Registers::SprTlr).await?;
        let original_spr_b = self.read_register(Channel::B, Registers::SprTlr).await?;

        // Test Channel A
        self.write_register(Channel::A, Registers::SprTlr, TEST_CHAR)
            .await?;
        let read_back_a = self.read_register(Channel::A, Registers::SprTlr).await?;

        // Test Channel B
        self.write_register(Channel::B, Registers::SprTlr, !TEST_CHAR)
            .await?;
        let read_back_b = self.read_register(Channel::B, Registers::SprTlr).await?;

        // Restore original values
        self.write_register(Channel::A, Registers::SprTlr, original_spr_a)
            .await?;
        self.write_register(Channel::B, Registers::SprTlr, original_spr_b)
            .await?;

        Ok(read_back_a == TEST_CHAR && read_back_b == !TEST_CHAR)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
