use cortex_m_semihosting::hprintln;
use stm32g4xx_hal::{gpio::{self, AF10}, quadspi::{ClockMode, Command, DdrMode, FlashMode, IoCommand, LineMode, Qspi, QuadSpiExt}, rcc::Rcc};
use stm32g4xx_hal::pac;

type NcsPin = gpio::PA2<AF10>;
type ClkPin = gpio::PA3<AF10>;

type Io0Bank1Pin = gpio::PB1<AF10>;
type Io1Bank1Pin = gpio::PB0<AF10>;
type Io2Bank1Pin = gpio::PA7<AF10>;
type Io3Bank1Pin = gpio::PA6<AF10>;

type Io0Bank2Pin = gpio::PC1<AF10>;
type Io1Bank2Pin = gpio::PC2<AF10>;
type Io2Bank2Pin = gpio::PC3<AF10>;
type Io3Bank2Pin = gpio::PC4<AF10>;

pub struct STM32G4xxQspiDriver {
    qspi_bus: Qspi,
}

impl STM32G4xxQspiDriver {
    pub fn new(
        qspi: pac::QUADSPI,
        rcc: &mut Rcc,
        ncs_pin: NcsPin,
        clk_pin: ClkPin,
        io0_bank1_pin: Io0Bank1Pin,
        io1_bank1_pin: Io1Bank1Pin,
        io2_bank1_pin: Io2Bank1Pin,
        io3_bank1_pin: Io3Bank1Pin,
        io0_bank2_pin: Io0Bank2Pin,
        io1_bank2_pin: Io1Bank2Pin,
        io2_bank2_pin: Io2Bank2Pin,
        io3_bank2_pin: Io3Bank2Pin,
    ) -> STM32G4xxQspiDriver {
        let mut res = Self {
            qspi_bus: qspi.qspi(
                (
                    clk_pin,
                    io0_bank1_pin, io1_bank1_pin, io2_bank1_pin, io3_bank1_pin, ncs_pin,
                    io0_bank2_pin, io1_bank2_pin, io2_bank2_pin, io3_bank2_pin
                ), 
                rcc,
                0, 8, false,
                21, 2, ClockMode::Mode0, FlashMode::Dual
            )
        };

        // Continuous Read Mode Reset
        res.qspi_bus.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0xFF)
        );

        // Write Enable for Volatile Status Register
        res.qspi_bus.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x50)
        );

        // Write Status Register (EQ=1)
        res.qspi_bus.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x31)
            .with_alternate_bytes(LineMode::Single, [0x02])
        );

        // Enable QPI
        res.qspi_bus.command(Command::new(DdrMode::Disabled)
            .with_instruction(LineMode::Single, 0x38)
        );

        // Read Identification Sequence
        let mut id_seq: [u8; 6] = [0,0,0,0,0,0];
        res.qspi_bus.read(IoCommand::new(DdrMode::Disabled, LineMode::Quad)
            .with_instruction(LineMode::Quad, 0x9F),
            &mut id_seq
        );
        if id_seq != [0xBA,0xBA,0x60,0x60,0x15,0x15] {
            hprintln!("Invalid flash ID sequence: {:x?}", id_seq)
        }

        res
    }

    pub fn chip_erase(&mut self) {
        todo!()
    }

    pub fn page_write(&mut self, page_index: u32, data: &[u8; 512]) {
        todo!()
    }

    pub fn enabled_mapping(self) {
        todo!()
    }
}
