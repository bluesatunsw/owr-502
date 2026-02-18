# VESC Firmware 

## Running and debugging the firmware

To run the firmware in a debugging/development context with an ST-LINK connected:

```
openocd -f ../embedded_common/tools/openocd-stm32f4.cfg
cargo run --release
```

It's pretty much necessary to run in release mode due to space optimisations
being necessary to fit onto the dozens of KB of device flash.

To flash the firmware *and have it continue to run after you disconnect the
debugger* (for integration testing, or the competition (!)), do *not* run
OpenOCD. Instead, with the debugger connected, run:

```
cargo flash --release --chip stm32f446mc
```

(You may need to install `cargo flash`.)


## Current Sense Timings

DRV8301 is config'd thru resistor for 80ns deadtime
50 dts ticks @ 168MHz --> 298ns

TIM1 is on APB2, TIM1 CLK -> 168MHz, TIM1.ARR -> 6000

TIM1.CNT      0 -----up to------>  TIM.ARR   -----down to----> 0

              <------------------  35.71μs  ------------------->
TIM1.OCxREF   ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺\____________/⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺
TIM1.OCx      ⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺\______________/⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺
TIM1.OCxN     ____________________/⎺⎺⎺⎺⎺⎺⎺⎺⎺⎺\__________________
                                 <> deadtime  <> 298ns
               at least 595ns   <------------> max duty limit 
TIM1.UEV      \________________________________________________/
^^^ commutation (+current PID control) interrupt (TIM1.RCR = 1, so every cycle)


ADCCLK = APB2CLK (84MHz) / 4 == 21MHz (max 36MHz)
                          vvvvv NOT TO SCALE vvvvv
ADC Samp      ______________________⎺___________________________
                                   ^ sample time 3 clks == 143ns
                                    ^ inj group trigger latency 3 clks == 143ns
ADC Conv      _______________________/⎺⎺\_______________________
               12 clks == 571ns      <-->           (doesn't affect timing 😸)

## Duty cycle limit
We need to limit the duty cycle to ensure channel A,B always have some low time
(to allow for current sensing). Assuming 0 settling time after switch

Tcurrsense = deadtime + ADC inj trig latency + ADC samp time
= 442ns (75 TIM cnt) 
~= 100 TIM1 counts (w/ settling time) (595ns) => 98.3% max duty


Aside: rust stm32f4xx-hal uses output-compare mode 1
```
PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
else inactive. In downcounting, channel 1 is inactive (OC1REF=0) as long as
TIMx_CNT>TIMx_CCR1 else active (OC1REF=’1’).
```
