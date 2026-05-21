
# Arduino

Embedded firmware for the FWMAV motor drive system. Controls two brushed DC motors via an Adafruit TB6612FNG dual H-bridge, driven by a sine-wave look-up table stored in program memory. Runs an automated frequency sweep from a configurable start frequency to an end frequency, logging each step over Serial.

---

## Hardware

| Component | Part |
|---|---|
| Microcontroller | Arduino Nano Every |
| Motor driver | Adafruit TB6612FNG |
| Motors | Precision Microdrives 206-10K (×2) |
| Supply | Bench PSU, 4.5 V |

**Pin assignment**

| Signal | Pin |
|---|---|
| AIN1 / AIN2 | 2, 4 |
| PWMA | 9 |
| BIN1 / BIN2 | 8, 7 |
| PWMB | 10 |
| STBY | 6 |

---

## Files

| File | Description |
|---|---|
| `Wing_Flappingv3.ino` | Main firmware: sine LUT, Timer2 ISR, frequency sweep loop |

---

## How It Works

A 128-point half-sine look-up table is stored in `PROGMEM`. Timer2 fires an ISR at the required step rate; each interrupt advances the phase counter, reads the next PWM value from the LUT, and sets the H-bridge direction pins to produce a full sinusoidal cycle across 256 steps. Motor B is driven 180° out of phase with Motor A so the two wings flap symmetrically in opposition.

The `setFrequency()` function selects the Timer2 prescaler (÷64 or ÷1024) and computes the output-compare register value to hit the requested frequency with minimal rounding error.

---

## Configuration

Edit the constants at the top of `Wing_Flappingv3.ino`:

```cpp
const float START_FREQ     = 2.0;    // Hz
const float END_FREQ       = 50.0;   // Hz
const float STEP_FREQ      = 2.0;    // Hz increment
const unsigned long DWELL_MS    = 3000;  // run time per step (ms)
const unsigned long STOP_MS     = 1000;  // pause between steps (ms)
const unsigned long START_DELAY_MS = 3000;  // initial wait (ms)
```

---

## Serial Output

Open the Serial Monitor at 9600 baud. The firmware prints the start and end of each frequency step, making it straightforward to correlate with a simultaneously running DAQ recording.

```
Waiting 3 seconds before sweep...
Sweep starting.
FREQ 2.0 Hz - START
FREQ 2.0 Hz - END
STOPPED
FREQ 4.0 Hz - START
...
Sweep complete.
```
