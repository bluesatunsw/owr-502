use stm32g4::stm32g474::{self, *};
use stm32g4xx_hal::{
    dma::channel::{self, DMAExt}, flash::{FlashExt, Parts}, gpio::*, pwr::PwrExt, rcc::{Config, Enable, FdCanClockSource, PllConfig, PllMDiv, PllNMul, PllQDiv, PllRDiv, PllSrc, Rcc, RccExt, Reset}, time::RateExtU32
};

pub type ClockTim = TIM2;

pub type DmaChannel = channel::C<DMA1, 0>;

pub type CanInstance = FDCAN1;
pub type CanRxPin = PA11<AF9>;
pub type CanTxPin = PA12<AF9>;

pub type QspiNcsPin = PA2<AF10>;
pub type QspiClkPin = PA3<AF10>;

pub type QspiIo0Bank1Pin = PB1<AF10>;
pub type QspiIo1Bank1Pin = PB0<AF10>;
pub type QspiIo2Bank1Pin = PA7<AF10>;
pub type QspiIo3Bank1Pin = PA6<AF10>;

pub type QspiIo0Bank2Pin = PC1<AF10>;
pub type QspiIo1Bank2Pin = PC2<AF10>;
pub type QspiIo2Bank2Pin = PC3<AF10>;
pub type QspiIo3Bank2Pin = PC4<AF10>;

pub type ArgbInstance = USART1;
pub type ArgbPin = PB6<AF7>;

pub struct Peripherals {
    pub sys: SYSCFG,
    pub rcc: Rcc,

    pub clock_tim: ClockTim,

    pub crc_instance: CRC,
    pub dma_chan: DmaChannel,

    pub can_instance: CanInstance,
    pub can_rx_pin: CanRxPin,
    pub can_tx_pin: CanTxPin,    

    pub argb_instance: ArgbInstance,
    pub argb_pin: ArgbPin,

    pub qspi_instance: QUADSPI,
    pub qspi_clk_pin: QspiClkPin,
    pub qspi_ncs_pin: QspiNcsPin,

    pub qspi_io0_bank1_pin: QspiIo0Bank1Pin,
    pub qspi_io1_bank1_pin: QspiIo1Bank1Pin,
    pub qspi_io2_bank1_pin: QspiIo2Bank1Pin,
    pub qspi_io3_bank1_pin: QspiIo3Bank1Pin,

    pub qspi_io0_bank2_pin: QspiIo0Bank2Pin,
    pub qspi_io1_bank2_pin: QspiIo1Bank2Pin,
    pub qspi_io2_bank2_pin: QspiIo2Bank2Pin,
    pub qspi_io3_bank2_pin: QspiIo3Bank2Pin,

    pub internal_flash: Parts,
}

impl Peripherals {
    pub fn take() -> Self {
        // SAFETY: This can/should only be called right at the start of main
        let dp = unsafe { stm32g474::Peripherals::take().unwrap_unchecked() };

        let pwr = dp.PWR.constrain().freeze();

        let mut rcc = dp.RCC.freeze(
            Config::pll()
                .pll_cfg(PllConfig {
                    mux: PllSrc::HSE(24.MHz()),
                    m: PllMDiv::DIV_3,
                    n: PllNMul::MUL_32,
                    r: Some(PllRDiv::DIV_4),
                    q: Some(PllQDiv::DIV_2),
                    p: None,
                })
                .fdcan_src(FdCanClockSource::PLLQ),
            pwr
        );

        let gpioa = dp.GPIOA.split(&mut rcc);
        let gpiob = dp.GPIOB.split(&mut rcc);
        let gpioc = dp.GPIOC.split(&mut rcc);

        let dma1 = dp.DMA1.split(&rcc);

        Self {
            sys: dp.SYSCFG,
            rcc: rcc,

            clock_tim: dp.TIM2,

            crc_instance: dp.CRC,
            dma_chan: dma1.ch1,

            can_instance: dp.FDCAN1,
            can_rx_pin: gpioa.pa11.into_alternate(),
            can_tx_pin: gpioa.pa12.into_alternate(),

            argb_instance: dp.USART1,
            argb_pin: gpiob.pb6.into_alternate(),

            qspi_instance: dp.QUADSPI,
            qspi_clk_pin: gpioa.pa3.into_alternate().speed(Speed::VeryHigh),
            qspi_ncs_pin: gpioa.pa2.into_alternate().speed(Speed::VeryHigh),

            qspi_io0_bank1_pin: gpiob.pb1.into_alternate().speed(Speed::VeryHigh),
            qspi_io1_bank1_pin: gpiob.pb0.into_alternate().speed(Speed::VeryHigh),
            qspi_io2_bank1_pin: gpioa.pa7.into_alternate().speed(Speed::VeryHigh),
            qspi_io3_bank1_pin: gpioa.pa6.into_alternate().speed(Speed::VeryHigh),

            qspi_io0_bank2_pin: gpioc.pc1.into_alternate().speed(Speed::VeryHigh),
            qspi_io1_bank2_pin: gpioc.pc2.into_alternate().speed(Speed::VeryHigh),
            qspi_io2_bank2_pin: gpioc.pc3.into_alternate().speed(Speed::VeryHigh),
            qspi_io3_bank2_pin: gpioc.pc4.into_alternate().speed(Speed::VeryHigh),

            internal_flash: dp.FLASH.constrain(),
        }
    }

    pub unsafe fn reset() {
        unsafe {
            ClockTim::reset_unchecked();
            ClockTim::disable_unchecked();

            CRC::reset_unchecked();
            CRC::disable_unchecked();

            DMA1::reset_unchecked();
            DMA1::disable_unchecked();

            CanInstance::reset_unchecked();
            CanInstance::disable_unchecked();

            ArgbInstance::reset_unchecked();
            ArgbInstance::disable_unchecked();

            // Steal and poke the GPIO registers since the systems have taken ownership
            // of them (and there is nothing which can be done about that)

            GPIOA::steal().afrh().modify(
                |_, w| w
                    .afrh11().set(0)
                    .afrh12().set(0)
            );
            GPIOA::steal().moder().modify(
                |_, w| w
                    .moder11().analog()
                    .moder12().analog()
            );

            GPIOB::steal().afrl().modify(
                |_, w| w
                    .afrl6().set(0)
            );
            GPIOB::steal().moder().modify(
                |_, w| w
                    .moder6().analog()
            );
        }
    }
}
