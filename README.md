# Programmable Arduino DCC Turntable Controller
This is heavily based on the "DCCInterface_TurntableControl" example by Alex Shepherd included with the NmraDCC library, adapted to suit the ULN2003 stepper controller, my need for automatic polarity reversal of the bridge track, and the desire to be able to program the turntable positions via DCC commands.

This repository contains the code for a programmable Arduino based turntable controller utilising a ULN2003 stepper motor controller and 28BYJ-48 stepper motor.

The turntable is driven by DCC commands according to the NMRA standards and, as such, requires an external DCC decoder interface circuit.

The decoder address CVs can be programmed as per a normal DCC accessory decoder, and the code is written to provide the DCC ACK signal when programming.

In addition, the turntable positions and polarity flag are also programmed as per a normal DCC accessory decoder, and these are stored in the manufacturer unique CVs starting at 513.

The bridge track polarity is able to be switched via a generic Arduino dual relay board to avoid needing to use an auto-reverser.

This has been tested on a generic Arduino Uno R3 with a Mega2560 based DCC++ EX CommandStation.

# Instructions

The 28BYJ-48 is an inexpensive unipolar stepper motor used in many consumer devices and is an ideal candidate for a small scale turntable. The motor is driven by the ULN2003 controller using 4 pins as outputs from the Arduino in full step mode.

The full step pin driving order is 1, 3, 2, 4 to drive anti-clockwise, and 4, 2, 3, 1 to drive clockwise.

A hall effect sensor is combined with a magnet attached to one end of the turntable bridge to detect the home position in order to maintain calibration and alignment with the various track positions.

## Defining positions

**IMPORTANT** There is a hidden feature whereby you can send an accessory (a) command to the decoder address one above the highest defined turnout position to cause the Arduino to reset. This is handy to trigger a reset after programming on the main track. It's important to note this to avoid putting any other accessories in service utilising the decoder address.

The positions are defined in the reserved manufacturer unique CVs from 513 to 895.

CV 513 holds the number of positions that are to be defined (maximum of 50).

Each position requires three CVs to be programmed:
- Least significant bit (LSB) of the step count from the home position
- Most significant bit (MSB) of the step count from the home position
- Polarity switch flag: 0 to maintain, 1 to switch polarity - without this, an auto-reverser would be required to keep polarity in sync between the bridge and surrounding tracks

An example table of position definitions for CV programming for a turntable with six positions and a base DCC decoder defined as 201:

CV 513 = 6

| Position | DCC Address | Steps from Home | LSB CV | Value | MSB CV | Value | Polarity CV | Value |
|---|---|---|---|---|---|---|---|---|
| 1 | 201 | 12 | 514 | 12 | 515 | 0 | 516 | 0 |
| 2 | 202 | 12 | 517 | 12 | 518 | 0 | 519 | 0 |
| 3 | 203 | 12 | 520 | 12 | 521 | 0 | 522 | 0 |
| 4 | 204 | 12 | 523 | 12 | 524 | 0 | 525 | 0 |
| 5 | 205 | 12 | 526 | 12 | 527 | 0 | 528 | 0 |
| 6 | 206 | 12 | 529 | 12 | 530 | 0 | 531 | 0 |
| Reset | 207 | NA | NA | NA | NA | NA | NA | NA |

# Arduino pins
The code uses the pins below by default for the various functions, adjust as suits your needs.

- D2 - Interrupt pin (input) used for decoding the DCC commands
- A1 - Output used for the sending the DCC ACK signal when programming
- D3 - Input used for the hall effect sensor to detect the home position
- D4 - Relay 1 control pin
- D5 - Relay 2 control pin
- D8  - ULN2003 IN1
- D9  - ULN2003 IN2
- D10 - ULN2003 IN3
- D11 - ULN2003 IN4
