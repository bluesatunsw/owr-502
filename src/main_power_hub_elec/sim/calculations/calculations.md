# Main Power Module
z5492250 Jethro Rosettenstein

Friday August 29, 2025

4-channel switching power mux

Design resources:

- https://www.ti.com/lit/an/slva398a/slva398a.pdf?ts=1756423659469&ref_url=https%253A%252F%252Fduckduckgo.com%252F

- https://www.ti.com/lit/ml/slua618a/slua618a.pdf?ts=1756010773104&ref_url=https%3A%2F%2Fwww.google.com%2F

# Driver
Pullup configuration with BJT

## Design considerations:
- Loss, depends on
- Time to switch
  Ideally want as slow as possible such that the maximum instantaneous power is not exceeded
- Collector resistor value
  Needs to drop the power MOSFET gate voltage below Vsource + Vtn
- Assume Vdrv will be provided from the discrete charge pump at ~36V

## Power MOSFET: C2902884
https://www.lcsc.com/datasheet/C2902884.pdf

Maximum junction temperature = 150°C

Qgd = 28nC

Miller plateau voltage = 3.75V

Rise time = 38ns

During switch-on, power losses happen while $$V_{GS}$$ charges and then when $$V_{DS}$$ falls (see Figure 4).

### T2:
Voltage = $$V_{DS(off)}$$

Current = $$\frac{I_D}{2}$$

Instantaneous power = $$V_{DS(off)} \frac{I_D}{2}$$

Input capacitance charges from $$V_{tn}$$ to $$V_{GS(miller)}$$

Therefore $$t_2 = C_{ISS} \frac{VGS(Miller) - Vtn}{I_{G2}}$$

We can estimate the gate current from the MOSFET equation (triode region):

$$I_{G2} = \frac{V_{DRV} - 0.5 \times (V_{GS (miller)} + V_{TH})}{R_{HI} + R_{GATE} _ R_{G,I}}$$

Denominator is just the summed input resistance of the MOSFET.

### T3:
Voltage = $$\frac{V_{DS(off)}}{2}$$

Current = $$I_D$$

Instantaneous power = $$V_{DS(off)} \frac{I_D}{2}$$

Miller capacitance discharges from VDS(off) to ~0V

Therefore $$t_3 = C_{RSS} \frac{V_{DS(off)}}{I_{G3}}$$

Similarly, for the gate current, as we are in saturation region:

$$I_{G3} = \frac{V_{DRV} - V_{GS(miller)}}{R_{HI} + R_{GATE} + R_{G,I}}$$

In conclusion, we have the switching time and power of stages 2 and 3 of switching on.


### Time calculation
$$V_{DS(off)} = 30V$$ (to be safe) and $$I_D$$ should be designed for 40A maximum.

Therefore, the power dissipation during stages 2 and 3 is 600W.

If we use the normalized thermal impedance graph on the datasheet we arrive at 1ms maximum switching
time, and will select 500us as a safe bet.
