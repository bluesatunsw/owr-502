use stm32f4xx_hal::gpio::{Pin, PinState, Speed};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::timer::{Event, IdleState, PwmChannel};
use stm32f4xx_hal::{pac::TIM1, rcc::Rcc};

use crate::config::IdleMode;
use crate::util::motor_disable;

pub struct STM32F4xxSixPwmDriver {
    ph_a: PwmChannel<TIM1, 0, true>,
    ph_b: PwmChannel<TIM1, 1, true>,
    ph_c: PwmChannel<TIM1, 2, true>,
}

impl STM32F4xxSixPwmDriver {
    // Cap max duty across all phases since we don't have current sense on phase C
    // -> we always need to be able to measure A & B currents (on low-side!)
    // -> A or B can never reach 100% duty (to reconstruct Iq and Id)
    // -> Cap all of them because capping just A&B would be cursed (torque ripple)
    // 168MHz / 28kHz = 6000, 100 derived in README.md
    const PWM_SENSE_LIMIT: f32 = 6000.0 - 100.0;

    pub fn setup(
        rcc: &mut Rcc,
        tim: TIM1,
        pin_ah: Pin<'A', 8>,
        pin_al: Pin<'B', 13>,
        pin_bh: Pin<'A', 9>,
        pin_bl: Pin<'B', 14>,
        pin_ch: Pin<'A', 10>,
        pin_cl: Pin<'B', 15>,
        mode: IdleMode,
    ) -> Self {
        // trig UEV on *full* pwm cycle (1 bcuz center aligned :sob:)
        tim.rcr().write(|w| w.rep().set(1));

        // Actually 14kHz since we're in center aligned mode!
        // ⚠️ Change dead_time and PWM_SENSE_LIMIT if changing this! ⚠️
        let (mut pwm_mgr, (ch1, ch2, ch3, ..)) = tim.pwm_hz(28.kHz(), rcc);
        assert!(pwm_mgr.get_max_duty() == 6000);

        motor_disable();
        pwm_mgr.set_dead_time(50);
        pwm_mgr.set_cms(stm32f4xx_hal::timer::CenterAlignedMode::CenterAligned1);
        pwm_mgr.listen(Event::Update); // Commutation handler, needs to be NVIC unmasked!

        let mut pwm_c1 = ch1
            .with(
                pin_ah
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            )
            .with_complementary(
                pin_al
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            );
        pwm_c1.set_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);
        pwm_c1.set_complementary_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);

        let mut pwm_c2 = ch2
            .with(
                pin_bh
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            )
            .with_complementary(
                pin_bl
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            );
        pwm_c2.set_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);
        pwm_c2.set_complementary_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);

        let mut pwm_c3 = ch3
            .with(
                pin_ch
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            )
            .with_complementary(
                pin_cl
                    .into_push_pull_output_in_state(PinState::Low)
                    .speed(Speed::VeryHigh),
            );
        pwm_c3.set_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);
        pwm_c3.set_complementary_polarity(stm32f4xx_hal::timer::Polarity::ActiveHigh);

        Self::set_idle_mode(mode, (&mut pwm_c1, &mut pwm_c2, &mut pwm_c3));

        pwm_c1.enable();
        pwm_c1.enable_complementary();
        pwm_c2.enable();
        pwm_c2.enable_complementary();
        pwm_c3.enable();
        pwm_c3.enable_complementary();

        STM32F4xxSixPwmDriver {
            ph_a: pwm_c1,
            ph_b: pwm_c2,
            ph_c: pwm_c3,
        }
    }

    pub fn set_idle_mode(
        mode: IdleMode,
        ph: (
            &mut PwmChannel<TIM1, 0, true>,
            &mut PwmChannel<TIM1, 1, true>,
            &mut PwmChannel<TIM1, 2, true>,
        ),
    ) {
        ph.0.set_idle_state(IdleState::Reset);
        ph.1.set_idle_state(IdleState::Reset);
        ph.2.set_idle_state(IdleState::Reset);

        match mode {
            IdleMode::Ground => {
                ph.0.set_complementary_idle_state(IdleState::Set);
                ph.1.set_complementary_idle_state(IdleState::Set);
                ph.2.set_complementary_idle_state(IdleState::Set);
            }
            IdleMode::HiZ => {
                ph.0.set_complementary_idle_state(IdleState::Reset);
                ph.1.set_complementary_idle_state(IdleState::Reset);
                ph.2.set_complementary_idle_state(IdleState::Reset);
            }
        }
    }

    // 1. x = x+1 --> shift [-1, 1] to [0, 2]
    // 2. mul (16 bit wide TIMx.ARR capped for curr sense) --> [0, 11800], uncapped is 12000
    //  - Cap all of them equally cause capping just 2 would be goofy asf
    // 3. div 2 --> [0, 5900] A.K.A [0%, 98.3%]
    pub fn set_duty(&mut self, duties: [f32; 3]) {
        let v_abc_norm = duties.map(|x| ((x + 1.0) * (Self::PWM_SENSE_LIMIT / 2.0)) as u16);

        self.ph_a.set_duty(v_abc_norm[0]);
        self.ph_b.set_duty(v_abc_norm[1]);
        self.ph_c.set_duty(v_abc_norm[2]);
    }
}
