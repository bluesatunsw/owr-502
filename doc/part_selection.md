# Part Selection

## General Criteria

### Availability
- For the purpose of this criterion, close substitutes (replacements which require no modification to the design) are included.
- The part should have significant quantities in stock at at least two suppliers (LCSC, TME, Digi-Key, Mouser, etc.).
- Parts should not be significantly more expensive than others in the same category without justification.

### Reliability
- "No-name" brands should be avoided. Note: many Chinese companies have extremely janky websites and poor brand recognition despite producing significant quantities of reasonably high-quality parts (e.g. CCTC capacitors).
- Devices exposed to unregulated battery voltage should be rated to at least 50 V to handle transients.
- Capacitors should be operated at half their rated voltage, as large DC biases reduce the capacitance of MLCCs and ripple can destroy electrolytics.
- Maximum power dissipation ratings must be taken into account, not just maximum current ratings, for power components.

### Versatility
- If a part with more capabilities (e.g. higher voltage/current ratings) is available in a similarly sized package for a similar price, it should be chosen instead.

### Usability
- Avoid through-hole components, as they complicate assembly.
- For signal integrity, layout, and packaging reasons, the smallest package which can still be hand-soldered should be used.
- The smallest size of passive component that can easily be hand-soldered is 0402.
- Do not use BGAs or QFNs with pads completely under the package (they should extend slightly beyond the edge).

## Generic Components
The vast majority of components should come from the list of generic components. This allows us to:
- simplify logistics, reducing cost and lead time; and
- reduce the chance of being caught out by an unexpected quirk of a new component.

If there is already a similar component on the list, the reason it is not appropriate should be documented on the schematic. If there is not a similar component on the list, consider adding it.

For the purposes of procurement, BOMs do not need to include items on these lists.

### MLCCs
MLCCs are extremely small and cheap, but they unfortunately have significant variations in their capacitance. Most applications will work with a wide range of capacitance values, so two values per decade were deemed sufficient.

Higher capacitance values are also supplied with lower voltage variants to reduce size and cost.

#### 50 V MLCCs
| Value | Size | Part Number           |
| ----- | ---- | --------------------- |
| 10p   | 0402 | `CC0402JRNPO9BN100`   |
| 33p   | 0402 | `CC0402JRNPO9BN330`   |
| 100p  | 0402 | `CC0402JRNPO9BN101`   |
| 330p  | 0402 | `CC0402JRNPO9BN331`   |
| 1n    | 0402 | `CC0402JRNPO9BN102`   |
| 3n3   | 0402 | `CC0402KRX7R9BB332`   |
| 10n   | 0402 | `CC0402KRX7R9BB103`   |
| 33n   | 0402 | `CC0402KRX7R9BB333`   |
| 100n  | 0402 | `CC0402KRX7R9BB104`   |
| 330n  | 0603 | `TCC0603X7R334K500CT` |
| 1u    | 0805 | `TCC0805X7R105M500FT` |
| 10u   | 1206 | `TCC1206X5R106K500HT` |

#### 25 V MLCCs
| Value | Size | Part Number           |
| ----- | ---- | --------------------- |
| 330n  | 0402 | `TCC0402X5R334K250AT` |
| 1u    | 0402 | `TCC0402X5R105M250AT` |
| 3u3   | 0603 | `TCC0603X5R335M250CT` |
| 10u   | 0603 | `TCC0603X5R106K250CT` |

### Resistors
0402 1% tolerance resistors have been selected with three values per decade (E3 series). They are rated up to 0.125 W of average dissipation. More specific resistance values (e.g. for voltage regulator feedback) can be obtained using parallel and series combinations. KiCad has a calculator for finding these.

| Value | Part Number        |
| ----- | ------------------ |
| 1R    | `RC0402FR-071RL`   |
| 2R2   | `RC0402FR-072R2L`  |
| 4R7   | `RC0402FR-074R7L`  |
| 10R   | `RC0402FR-0710RL`  |
| 22R   | `RC0402FR-0722RL`  |
| 47R   | `RC0402FR-0747RL`  |
| 100R  | `RC0402FR-07100RL` |
| 220R  | `RC0402FR-07220RL` |
| 470R  | `RC0402FR-07470RL` |
| 1k    | `RC0402FR-071KL`   |
| 2k2   | `RC0402FR-072K2L`  |
| 4k7   | `RC0402FR-074K7L`  |
| 10k   | `RC0402FR-0710KL`  |
| 22k   | `RC0402FR-0722KL`  |
| 47k   | `RC0402FR-0747KL`  |
| 100k  | `RC0402FR-07100KL` |
| 220k  | `RC0402FR-07220KL` |
| 470k  | `RC0402FR-07470KL` |
| 1M    | `RC0402FR-071ML`   |

### Connectors
**[TODO]**

### ICs
**[TODO] See Common Board Standard**

### Other
**[TODO] See Common Board Standard**
