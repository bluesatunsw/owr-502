"""
z5492250 Jethro Rosettenstein
Friday August 29, 2025

Main Power Module
"""
import pint
ureg = pint.UnitRegistry()

# Power MOSFET
vds_off = 30 * ureg.volt
id_max = 40 * ureg.amp
p_max = (vds_off * id_max / 2).to('watt')
print('Maximum transient power: ', p_max)

max_allowable_power = 100 * ureg.degK / (2 * (ureg.degK /ureg.watt))
print('Maximum allowable power: ', max_allowable_power)

normalized_thermal_factor = max_allowable_power / p_max
print('Thermal factor: ', normalized_thermal_factor)

# looking at the graph (assuming lowest possible duty cycle as we should not be switching) we get
# 1ms switching time, so we sill use 500us to be safe.
