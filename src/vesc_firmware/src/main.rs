#![feature(unsafe_cell_access)]
#![feature(never_type)]
#![no_std]
#![no_main]

use core::convert::TryInto;
use core::ptr;
use core::{cell::UnsafeCell, mem::MaybeUninit};

use canadensis_data_types::reg::udral::service::actuator;
use canadensis_data_types::uavcan::node::health_1_0::Health;
use canadensis_data_types::uavcan::node::mode_1_0::Mode;

use canadensis::{
    core::time::Clock,
    node::{
        self,
        data_types::{GetInfoResponse, Version},
        BasicNode, CoreNode,
    },
    requester::TransferIdFixedMap,
    Node,
};
use canadensis_bxcan::BxCanDriver;
use canadensis_can::{CanNodeId, CanReceiver, CanTransmitter, CanTransport};
use cortex_m::{interrupt::Mutex, iprintln};
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use fixed::types::I16F16;
use foc::{
    park_clarke::{inverse_park, RotatingReferenceFrame},
    pwm::{Modulation, SpaceVector},
};
use panic_semihosting as _;

use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, Resolution},
        Adc,
    },
    gpio::{AnyPin, GpioExt, Input, PinState, Speed},
    interrupt,
    pac::{SPI3, TIM1},
    prelude::*,
    rcc::*,
    signature::Uid,
};

extern crate alloc;
pub mod bsp;
pub mod comms;
pub mod config;
pub mod state;
pub mod utils;

use crate::{
    bsp::{clock::STM32F4xxCyphalClock, drv8301::DRV8301, six_pwm::STM32F4xxSixPwmDriver},
    comms::{CommSystem, CYPHAL_CONCURRENT_TRANSFERS, CYPHAL_NUM_SERVICES, CYPHAL_NUM_TOPICS},
    config::presets::*,
    state::ControlMode,
    utils::{cos, sin},
};

// TODO: Move these into some CommutationState thingo or smth
static G_HALLS: Mutex<UnsafeCell<MaybeUninit<(AnyPin<Input>, AnyPin<Input>, AnyPin<Input>)>>> =
    Mutex::new(UnsafeCell::new(MaybeUninit::uninit()));
static G_PWMS: Mutex<UnsafeCell<MaybeUninit<STM32F4xxSixPwmDriver>>> =
    Mutex::new(UnsafeCell::new(MaybeUninit::uninit()));
static G_CONTROL: Mutex<UnsafeCell<ControlMode>> =
    Mutex::new(UnsafeCell::new(ControlMode::Disabled));

// Global allocator -- required by canadensis.
#[global_allocator]
static G_HEAP: Heap = Heap::empty();

fn initialise_allocator() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 16000;
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { G_HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}

///////////
// Main. //
///////////
#[entry]
fn main() -> ! {
    initialise_allocator();

    let config = DRIVEBASE_FL_CONFIG;

    // Embedded boilerplate...
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hse(24.MHz())
            .sysclk(168.MHz())
            .hclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz()),
    );

    dp.DBGMCU.apb2_fz().modify(|_, w| {
        w.dbg_tim1_stop().set_bit() // Stops counting AND disables outputs!
    });

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let mut led_green = gpiob.pb0.into_push_pull_output();
    let mut led_red = gpiob.pb1.into_push_pull_output();
    led_red.set_high();

    unsafe {
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        dp.DBGMCU
            .cr()
            .modify(|_, w| w.trace_ioen().set_bit().trace_mode().bits(0b00));
        cp.ITM.lar.write(0xC5ACCE55); // Unlock ITM registers
        cp.ITM.tcr.write(0x00010005); // AdvTBus ID = 1, Global ITM enable
        cp.ITM.ter[0].write(0b111); // Enables STIM1 & STIM0
        cp.ITM.tpr.write(0b1); // Enables STIM[7:0]
    }

    let drv = DRV8301::<SPI3>::setup(
        gpiob.pb5.into_push_pull_output_in_state(PinState::Low),
        gpiob.pb7.into_floating_input(),
        gpioc.pc9.into_push_pull_output_in_state(PinState::High),
        gpioc.pc10.into_alternate().speed(Speed::VeryHigh),
        gpioc.pc11.into_alternate().speed(Speed::VeryHigh),
        gpioc.pc12.into_alternate().speed(Speed::VeryHigh),
        dp.SPI3,
        &mut rcc,
    );

    let pwm_driver = STM32F4xxSixPwmDriver::setup(
        &mut rcc, dp.TIM1, gpioa.pa8, gpiob.pb13, gpioa.pa9, gpiob.pb14, gpioa.pa10, gpiob.pb15,
    );

    // Has external 2k2 pullups
    let hall1 = gpioc.pc6.into_pull_up_input();
    let hall2 = gpioc.pc7.into_pull_up_input();
    let hall3 = gpioc.pc8.into_pull_up_input();

    // Set up hall-sensor interfacing timer (TIM5)
    //
    // Mode of operation described in RM0390 16.3.18
    // OR refer to RM0440 28.3.29

    // Set up ADC's for voltage and motor current sensing
    // Multi-ADC Simultaneous injected mode?? Need to look into this later...
    // FIXME
    Adc::new(
        dp.ADC1,
        true,
        AdcConfig::default()
            .clock(stm32f4xx_hal::adc::config::Clock::Pclk2_div_4) // 21MHz
            .resolution(Resolution::Twelve),
        &mut rcc,
    );

    //dp.ADC_COMMON.ccr().write(|w| w.);

    // .external_trigger(adc::config::TriggerMode::RisingEdge, adc::config::ExternalTrigger::Tim_5_cc_1),
    //Adc::new(dp.ADC1, true, config, rcc);

    {
        let idx = (usize::from(hall1.is_high()) << 2)
            + (usize::from(hall2.is_high()) << 1)
            + usize::from(hall3.is_high());

        let angle = I16F16::FRAC_PI_3 * K_HALL_TABLE[idx];
        /*iprintln!(&mut cp.ITM.stim[0], "[INFO] HALL ANGLE: {}", angle);*/
        /*if idx == 0 || idx == 7 {
            iprintln!(
                &mut cp.ITM.stim[0],
                "[ERR] Invalid hall state {}... is the encoder plugged in?",
                idx
            );
        }*/
    }

    cortex_m::interrupt::free(|cs| unsafe {
        G_PWMS.borrow(cs).as_mut_unchecked().write(pwm_driver);
        G_HALLS
            .borrow(cs)
            .as_mut_unchecked()
            .write((hall1.into(), hall2.into(), hall3.into()));
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM1_UP_TIM10);
    }

    ///////////////////////////////////////////////////////////////////////////
    //                           CANADENSIS SETUP                            //
    ///////////////////////////////////////////////////////////////////////////

    let pcan = dp.CAN1.can((gpiob.pb9, gpiob.pb8), &mut rcc);
    let can = bxcan::Can::builder(pcan)
        // APB1: 42MHz, BTR: 1Mbps, Sample: 70%
        // BS1: 14, BS2: 6, BRP: 2 (21MHz), SJW: 2 ig...?
        .set_bit_timing(0x015D_0001)
        .enable();

    let clock = STM32F4xxCyphalClock::new(dp.TIM5, &mut rcc);
    let id: CanNodeId = config.comms.node_id.try_into().unwrap();
    let mut uuid: [u8; 16] = [0x00; 16];
    unsafe {
        ptr::copy_nonoverlapping(Uid::get() as *const Uid as *const u8, uuid.as_mut_ptr(), 12);
    }
    let driver = BxCanDriver::new(can);
    let transmitter = CanTransmitter::new(canadensis_can::Mtu::Can8);
    let receiver = CanReceiver::new(id);

    let core_node: node::CoreNode<
        _,
        _,
        _,
        TransferIdFixedMap<CanTransport, CYPHAL_CONCURRENT_TRANSFERS>,
        _,
        CYPHAL_NUM_TOPICS,
        CYPHAL_NUM_SERVICES,
    > = CoreNode::new(clock, id, transmitter, receiver, driver);

    let mut node = BasicNode::new(
        core_node,
        GetInfoResponse {
            protocol_version: Version { major: 1, minor: 0 },
            hardware_version: Version { major: 1, minor: 0 },
            software_version: Version { major: 1, minor: 0 },
            software_vcs_revision_id: 0,
            unique_id: uuid,
            name: heapless::Vec::from_slice(b"org.bluesat.owr.vesc").unwrap(),
            software_image_crc: heapless::Vec::new(),
            certificate_of_authenticity: heapless::Vec::new(),
        },
    )
    .unwrap();

    node.set_mode(Mode {
        value: Mode::OPERATIONAL,
    });

    node.subscribe_message(
        config.comms.ctrl_volt,
        size_of::<actuator::common::sp::scalar_0_1::Scalar>(),
        1.secs(),
    )
    .unwrap();

    let mut handler = CommSystem {
        control_mode: &G_CONTROL,
        config: config.comms,
    };

    let mut start = node.clock_mut().now();
    led_red.set_low();
    loop {
        if node.clock().advance_if_elapsed(&mut start, 1.secs()) {
            if drv.has_fault() {
                iprintln!(&mut cp.ITM.stim[0], "[WARN] DRV Fault!");
                node.set_health(Health {
                    value: Health::ADVISORY,
                });
            }

            if let Err(_) = node.run_per_second_tasks() {
                /*iprintln!(
                    &mut cp.ITM.stim[0],
                    "[WARN] node.run_per_second_tasks failed!"
                );*/
            }
            led_green.toggle();
        }

        if let Err(_) = node.receive(&mut handler) {
            /*iprintln!(&mut cp.ITM.stim[0], "[ERR] node.receive failed!");*/
        }
    }
}

const K_HALL_TABLE: [i32; 8] = [
    0, // 000 = xxx impossible
    3, // 001 = 3 (180 deg)
    5, // 010 = 5 (300 deg)
    4, // 011 = 4 (240 deg)
    1, // 100 = 1 (060 deg)
    2, // 101 = 2 (120 deg)
    0, // 110 = 0 (000 deg)
    0, // 111 = xxx impossible
];

#[interrupt]
fn TIM1_UP_TIM10() {
    unsafe {
        TIM1::steal().sr().modify(|_, w| w.uif().clear());
    }

    // Voltage expressed as a duty cycle... I don't care vro
    const V_D: I16F16 = I16F16::lit("0");
    let mut v_q: I16F16 = I16F16::lit("0"); // Duty cycle from -1 to 1

    cortex_m::interrupt::free(|cs| {
        let control = unsafe { G_CONTROL.borrow(cs).as_ref_unchecked() };

        if let ControlMode::Voltage { voltage: volts } = *control {
            v_q = volts;
        }
    });

    let mut angle: I16F16 = I16F16::ZERO;
    cortex_m::interrupt::free(|cs| unsafe {
        let halls = G_HALLS.borrow(cs).as_ref_unchecked().assume_init_ref();
        let idx = (usize::from(halls.0.is_high()) << 2)
            + (usize::from(halls.1.is_high()) << 1)
            + usize::from(halls.2.is_high());

        angle = I16F16::FRAC_PI_3 * K_HALL_TABLE[idx];

        while angle > I16F16::PI {
            angle -= I16F16::TAU;
        }
        while angle < -I16F16::PI {
            angle += I16F16::TAU;
        }
    });

    //unsafe {
    //    Peripherals::steal().ITM.stim[1].write_u32(transmute(angle));
    //}

    let (sin_angle, cos_angle) = (sin(angle), cos(angle));
    let v_ab: foc::park_clarke::TwoPhaseReferenceFrame = inverse_park(
        cos_angle,
        sin_angle,
        RotatingReferenceFrame { d: V_D, q: v_q },
    );
    // sinusoidal would do inverse clarke
    let v_abc = SpaceVector::modulate(v_ab);

    cortex_m::interrupt::free(|cs| unsafe {
        let pwm = G_PWMS.borrow(cs).as_mut_unchecked();
        pwm.assume_init_mut().set_duty(v_abc);
    });
}
