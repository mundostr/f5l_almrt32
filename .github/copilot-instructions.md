This project is devoted to develop a model aircraft altimeter board to be used specifically in a class called F5L.

All the class rules are here:
https://www.fai.org/sites/default/files/2026-06/CIAM-F5-Electric-26-June.pdf

All the specs to comply with when building to homologate the altimeter board are here:
https://www.fai.org/sites/default/files/2026-03/sc4_vol_edic_26.pdf

Please take those 2 docs as your main source of truth.

Here you have an extract of the specs taken from the EDIC doc:

F5L ALMRT

Specs:
- Altitude measurement: barometric pressure.
- Reference altitude: ground level altitude at the launch location (diferential).
- Altitude calculation: ISA (International Standard Atmosphere).
- Max launch altitude: 90 mts above launch ref.
- Max motor run: 30 secs.
- Motor stop conditions: 90 mts altitude, or 30 secs run, whichever occurs first.
- Manual motor stop: must be recognized.
- Motor restart throughout the flight: prohibited.
- User configuration: no settings that permit any operation outside the rules.
- Minimum sampling rate: 0.1 secs (10 hz).
- Minimum motor timing accuracy: 0.1 secs.
- Altitude accuracy: better than 2 mts measurement accuracy, +-2.5 mts motor stop height.
- The device must be installed in series between ESC and receiver.
- RC signal input: nominal 1.0 to 2.0 ms.
- Motor-on detection: signal exceeding 1.2 ms.
- Manual motor-off: signal below 1.2 ms.
- Initial motor-off output: must not exceed 0.9 ms.
- Motor stop output: must reliable prevent motor operation.
- Motor restart after stop: prohibited.

Desired sequence:
- Power: after powered, device inits and indicates it is armed.
- Throttle stick: for 2 secs device ignores any motor start signal level, maintaining motor output at 0.9 ms.
- Ground ref: it measures pressure at launch location (max init time: 2 secs).
- Motor ON state: after first receiver signal exceeding 1.2 ms, inits motor timer and altitude measurement.
- Flight monitoring: continuously monitors altitude and elapsed motor time.
- Motor OFF state: when receiver signal goes manually below 1.2 ms, the aircraft reaches 90 mts or the motor timer reaches 30 secs. This command is DEFINITIVE.
- Locked state: any subsequent motor commands are ignored, the motor cannot be restarted during that flight.

Altitude measurement details:
- By barometric pressure sensor, ISA atmospheric model, working in 750-1050 hPa and 0-50C temp range.
- Min accuracy: 2 m.
- Motor stop height accuracy: 2.5 mts.
- Min sampling freq: 10 hz (0.1 secs).
- Firmware calculation accuracy: +-0.5 mts.
- Sensor calibration: manufacturer calibration params must be incorporated.
- Sensor temperature compensation: according to manufacturer recommendations.
- ISA calculation formula to use, with P in hPa: h = 44330.76923 * (1 - (P / 1013.25)**0.190266669). Differential altitude is h(current pressure) - h(ground-reference pressure).
- Altitude calculation must be calculated at ground and at every new altitude reading.
- Filtering or smoothing must be applied to try to reject readings peaks due to motor activation, flight dynamics o just wind gusts.

Specs related to the programming environment:
- I use platformio as the base IDE.
- I need to program based on ESP-IDF framework.
- The base code must use FreeRTOS tasks for repetitive operations like time control or pressure reading.
- The code must be modular and well structured, with clear separation of concerns.
- When needed, generate separated files for functions or modules, so code remains clean.
- Do not document inside the code, i will do it manually.
- the main lopp must be driven based on a FSM (Finite State Machine).

Specs related to hardware used:
- Board: ESP32-C3 SuperMini.
- Pressure sensor: BMP280.