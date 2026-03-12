#![feature(unsafe_cell_access)]
#![feature(never_type)]
#![no_std]
#![no_main]
#![feature(more_float_constants)]
#![feature(const_trait_impl)]

use core::convert::TryInto;
use core::f32::consts;
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
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use embedded_alloc::LlffHeap as Heap;
use panic_semihosting as _;

use stm32f4xx_hal::pac::TIM3;
use stm32f4xx_hal::{
    adc::{
        config::{AdcConfig, Resolution},
        Adc,
    },
    gpio::{GpioExt, PinState, Speed},
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
pub mod util;

use crate::state::CommutationMode;
use crate::util::debug::itm_send_raw;
use crate::util::{
    debug::setup_itm,
    foc::{inverse_park, modulate_spacevector, RotatingReferenceFrame},
};
use crate::util::{motor_disable, motor_enable};
use crate::{
    bsp::{clock::STM32F4xxCyphalClock, drv8301::DRV8301, six_pwm::STM32F4xxSixPwmDriver},
    comms::{CommSystem, CYPHAL_CONCURRENT_TRANSFERS, CYPHAL_NUM_SERVICES, CYPHAL_NUM_TOPICS},
    config::presets::*,
    state::CommutationState,
};

static G_COM_STATE: Mutex<UnsafeCell<CommutationState>> =
    Mutex::new(UnsafeCell::new(CommutationState {
        mode: CommutationMode::Disabled,
        halls: MaybeUninit::uninit(),
        pwm_driver: MaybeUninit::uninit(),
        hall_state: 0.0,
        full_revs: 0,
        velocity_raw: 0.0,
    }));

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

    let config = DRIVEBASE_BR_CONFIG;

    // Embedded boilerplate...
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let mut dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();

    let mut rcc = dp.RCC.freeze(
        Config::hse(24.MHz())
            .sysclk(168.MHz())
            .hclk(168.MHz())
            .pclk1(42.MHz())
            .pclk2(84.MHz()),
    );

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let mut led_green = gpiob.pb0.into_push_pull_output();
    let mut led_red = gpiob.pb1.into_push_pull_output();
    led_red.set_high();

    // Set up debugging support
    unsafe {
        setup_itm(&mut cp.DCB, &mut cp.DWT, &mut dp.DBGMCU, &mut cp.ITM);
    }
    dp.DBGMCU.apb2_fz().modify(|_, w| {
        w.dbg_tim1_stop().set_bit() // Stops counting AND disables outputs!
    });

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
        &mut rcc,
        dp.TIM1,
        gpioa.pa8,
        gpiob.pb13,
        gpioa.pa9,
        gpiob.pb14,
        gpioa.pa10,
        gpiob.pb15,
        config.motion.idle_mode,
    );

    // Has external 2k2 pullups but ehhhh whatevs
    let hall1 = gpioc.pc6.into_alternate::<2>().internal_pull_up(true);
    let hall2 = gpioc.pc7.into_alternate::<2>().internal_pull_up(true);
    let hall3 = gpioc.pc8.into_alternate::<2>().internal_pull_up(true);

    // Set up hall-sensor interfacing timer (TIM3, 84MHz clock)
    //
    // Mode of operation described in RM0390 16.3.18
    // OR refer to RM0440 28.3.29
    TIM3::enable(&mut rcc);
    TIM3::reset(&mut rcc);
    // reset cr1 (pause), counter, pending interrupt flags
    dp.TIM3.cr1().reset();
    dp.TIM3.cnt().reset();
    dp.TIM3.sr().reset();
    // Set psc (time quanta = 1/28kHz) and arr (u16 max)
    dp.TIM3.arr().write(|w| w.arr().set(u16::MAX));
    dp.TIM3.psc().write(|w| {
        w.psc().set(
            (rcc.clocks.timclk1().raw() / 28.kHz::<1, 1>().raw())
                .try_into()
                .unwrap(),
        )
    });

    // Update on overflow so we can set velocity to 0, One punch mode
    dp.TIM3
        .cr1()
        .modify(|_, w| w.urs().counter_only().opm().enabled());
    // TI1/2/3 --[XOR]--> TI1 --> TI1F_ED
    dp.TIM3.cr2().write(|w| w.ti1s().xor());
    // TI1F_ED -> TRGI -> Reset CNT (Slave Mode)
    dp.TIM3
        .smcr()
        .write(|w| w.ts().ti1f_ed().sms().reset_mode());
    // Trigger an interrupt when the hall sensors change OR on timeout
    dp.TIM3.dier().write(|w| w.tie().enabled().uie().enabled());
    // Capture CNT on TRC event (actual hall time), filter
    dp.TIM3.ccer().reset();
    dp.TIM3
        .ccmr1_input()
        .write(|w| w.cc1s().trc().ic1f().fdts_div32_n8());
    dp.TIM3.ccer().write(|w| w.cc1e().enabled());

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

    cortex_m::interrupt::free(|cs| unsafe {
        let com_state = G_COM_STATE.borrow(cs).as_mut_unchecked();
        com_state.pwm_driver.write(pwm_driver);
        com_state
            .halls
            .write((hall1.into(), hall2.into(), hall3.into()));
        com_state.hall_state = com_state.get_angle_force_hw();
        dprintln!(0, "[INFO] Hall angle: {}", com_state.hall_state);
    });

    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM1_UP_TIM10);

        cp.NVIC.set_priority(interrupt::TIM3, 31);
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM3);
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
        config.comms.ctrl_duty,
        size_of::<actuator::common::sp::scalar_0_1::Scalar>(),
        1.secs(),
    )
    .unwrap();

    let mut handler = CommSystem {
        commutation_state: &G_COM_STATE,
        config: config.comms,
    };

    dprintln!(0, "Finished setup");

    let mut start = node.clock_mut().now();
    led_red.set_low();
    loop {
        if drv.has_fault() {
            motor_disable();
            node.set_health(Health {
                value: Health::WARNING,
            });
            dprintln!(0, "[WARN] DRV Fault!");
        }

        if node.clock().advance_if_elapsed(&mut start, 1.secs()) {
            if let Err(_) = node.run_per_second_tasks() {
                dprintln!(0, "[WARN] node.run_per_second_tasks failed!");
            }
            led_green.toggle();
        }

        if let Err(_) = node.receive(&mut handler) {
            dprintln!(0, "[ERR] node.receive failed!");
        }
    }
}

// Hall sensor interfacing interrupt
//
// This will NOT capture the velocity on the first hall transition after it
// times out, it requires another (first enables the counter) for a capture
#[interrupt]
fn TIM3() {
    // If we overflowed the timer period at some point
    let tim = unsafe { TIM3::steal() };
    tim.cr1().modify(|_, w| w.cen().enabled());

    /*if tim.sr().read().cc1of().is_overcapture() {
        dprintln!(0, "[ERR] Hall overcapture!");
    }*/

    // Get old and new hall sensor states to check if we need to wrap around
    // AND wtf the direction of our velocity is!
    let (halls_old, halls_new) = cortex_m::interrupt::free(|cs| unsafe {
        let com_state = G_COM_STATE.borrow(cs).as_mut_unchecked();
        (com_state.hall_state, com_state.get_halls())
    });

    // This is still fine in the case of UIF firing (timer overflow)
    let (direction, wrap) = match (halls_old, halls_new) {
        (state::K_HALL_MIN, state::K_HALL_MAX) => (-1, 1),
        (state::K_HALL_MAX, state::K_HALL_MIN) => (1, 1),
        _ if halls_new > halls_old => (1, 0),
        _ => (-1, 0),
    };

    // ticks (one per 1/28kHz) between hall transition --> speed (rad/s)
    // speed = (28k / ticks) * 1/6 (hall trans/turn * 2pi (turn/s->rad/s)
    let velocity = if tim.sr().read().uif().is_update_pending() {
        0.0
    } else if tim.ccr1().read().ccr().bits() <= 2u16 {
        // Assume hall sensor glitched
        cortex_m::interrupt::free(|cs| unsafe {
            G_COM_STATE.borrow(cs).as_mut_unchecked().glitch_accum +=
                tim.ccr1().read().ccr().bits();
        });
        return;
    } else {
        let glitches = cortex_m::interrupt::free(|cs| unsafe {
            G_COM_STATE.borrow(cs).as_mut_unchecked().glitch_accum
        });
        (28000.0 / ((tim.ccr1().read().ccr().bits() + glitches) as f32 * 6.0))
            * consts::TAU
            * direction as f32
    };
    tim.sr().reset();

    // Write new numbers
    cortex_m::interrupt::free(|cs| unsafe {
        let com_state = G_COM_STATE.borrow(cs).as_mut_unchecked();
        com_state.glitch_accum = 0; // Clear glitch velocity accumulator
        com_state.hall_state = halls_new;
        com_state.velocity_raw = velocity;
        com_state.full_revs += wrap * direction;

        itm_send_raw(1, &halls_new);
        itm_send_raw(2, &velocity);
        itm_send_raw(3, &com_state.full_revs);
    });
}

// Motion control interrupt

// Commutation interrupt
#[interrupt]
fn TIM1_UP_TIM10() {
    unsafe {
        TIM1::steal().sr().modify(|_, w| w.uif().clear());
    }

    if cortex_m::interrupt::free(|cs| {
        let control = unsafe { G_COM_STATE.borrow(cs).as_ref_unchecked() };
        match control.mode {
            CommutationMode::Disabled => {
                motor_disable();
                true
            }
            _ => {
                motor_enable();
                false
            }
        }
    }) {
        // Exit early if we're idle
        return;
    };

    // Voltage expressed as a duty cycle... I don't care vro
    const V_D: f32 = 0.0;
    // Duty cycle from -1 to 1
    let v_q: f32 = cortex_m::interrupt::free(|cs| {
        let control = unsafe { G_COM_STATE.borrow(cs).as_ref_unchecked() };
        if let CommutationMode::DutyCycle { duty } = control.mode {
            return duty;
        }

        0.0
    });

    let angle = cortex_m::interrupt::free(|cs| unsafe {
        G_COM_STATE
            .borrow(cs)
            .as_ref_unchecked()
            .get_angle_force_hw()
    });

    // Saving grace... at least these expensive ass math ops aren't in an int::free
    let v_ab = inverse_park(angle, RotatingReferenceFrame { d: V_D, q: v_q });
    let v_abc = modulate_spacevector(v_ab);

    cortex_m::interrupt::free(|cs| unsafe {
        let pwm = G_COM_STATE
            .borrow(cs)
            .as_mut_unchecked()
            .pwm_driver
            .assume_init_mut();
        pwm.set_duty(v_abc);
    });
}
