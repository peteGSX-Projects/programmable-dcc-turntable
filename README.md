# Arduino DCC Turntable Controller
**Important note:** This code is actively under development, use at your own peril! It is heavily based on the "DCCInterface_TurntableControl" example by Alex Shepherd included with the NmraDCC library, adapted to suit the ULN2003 stepper controller and my need for automatic polarity reversal of the bridge track.

This repository contains the code for an Arduino based turntable controller utilising a ULN2003 stepper motor controller and 28BYJ-48 stepper motor.

The turntable is driven by DCC commands according to the NMRA standards and, as such, requires an external DCC decoder interface circuit.

The DCC CVs can be programmed as per a normal DCC accessory decoder, and the code is written to provide the DCC ACK signal when programming.

The code complies with the NMRA turnout direction standard so that 1 = Closed, and 0 = Thrown.

In addition, the bridge track polarity is able to be switched via a generic Arduino dual relay board to avoid needing to use an auto-reverser.

# Instructions

The 28BYJ-48 is an inexpensive unipolar stepper motor used in many consumer devices and is an ideal candidate for a small scale turntable. The motor is driven by the ULN2003 controller using 4 pins as outputs from the Arduino in full step mode.

The full step pin driving order is 1, 3, 2, 4 to drive anti-clockwise, and 4, 2, 3, 1 to drive clockwise.

A hall effect sensor is combined with a magnet attached to one end of the turntable bridge to detect the home position in order to maintain calibration and alignment with the various track positions.

## Track positions

Track positions are defined as an array containing one or more C++ struct items to define the DCC address and steps from 0.

To configure turntable positions, the constant "maxTurntablePositions" (X) must be updated to reflect the total number of positions.

Once this is defined, each position requires an entry in the "turntablePositions" array as outlined below, numbered from 0 to the number of items minus one (X - 1), with the number of steps from the home position (Y).

So, if there are 10 turntable positions to be defined, X = 10, and the positions are entered as turntablePositions[0] through turntablePositions[X - 1], with the appropriate steps listed for each position (Y).

Code requiring modification:
```
const int maxTurntablePositions = X;

turntablePositions[0] = (turntablePosition) {baseTurntableAddress + 0, Y};
...
turntablePositions[X - 1] = (turntablePosition) {baseTurntableAddress + X - 1, Y};
```

Example for 4 positions:
```
const int maxTurntablePositions = 4;

turntablePositions[0] = (turntablePosition) {baseTurntableAddress + 0, 0};
turntablePositions[1] = (turntablePosition) {baseTurntableAddress + 1, 500};
turntablePositions[2] = (turntablePosition) {baseTurntableAddress + 2, 1000};
turntablePositions[3] = (turntablePosition) {baseTurntableAddress + 3, 2000};
```

# Arduino pins
The code uses the pins below for the various functions.

## DCC pins
- D2 - Interrupt pin (input) used for decoding the DCC commands
- D3 - Output used for the sending the DCC ACK signal when programming

## Home sensing pin
- D4 - Input used for the hall effect sensor to detect the home position

## ULN2003 pins
- D8  - ULN2003 IN1
- D9  - ULN2003 IN2
- D10 - ULN2003 IN3
- D11 - ULN2003 IN4
