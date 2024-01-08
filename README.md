# Programmable Arduino DCC Turntable Controller

*Note:* Since writing this initial software, I've changed tact for my layout and will be using a DCC-EX integrated turntable using the (EX-Turntable)[https://dcc-ex.com/ex-turntable/index.html] software instead. The code for this programmable DCC turntable is still available if it's of use for anyone, but unlikely to undergo any further development.

This is heavily based on the "DCCInterface_TurntableControl" example by Alex Shepherd included with the NmraDCC library, adapted to suit the ULN2003 stepper controller, my need for automatic polarity reversal of the bridge track, and the desire to be able to program the turntable positions via DCC commands.

This repository contains the code for a programmable Arduino based turntable controller utilising a ULN2003 stepper motor controller and 28BYJ-48 stepper motor.

The turntable is driven by DCC commands according to the NMRA standards and, as such, requires an external DCC decoder interface circuit.

The decoder address CVs can be programmed as per a normal DCC accessory decoder, and the code is written to provide the DCC ACK signal when programming.

In addition, the turntable positions and polarity flag are also programmed as per a normal DCC accessory decoder, and these are stored in the manufacturer unique CVs starting at 513.

The bridge track polarity is able to be switched via a generic Arduino dual relay board to avoid needing to use an auto-reverser.

This has been tested on a generic Arduino Uno R3 with a Mega2560 based DCC++ EX CommandStation.

For further information, please refer to my [GitHub pages](https://petegsx-projects.github.io/index.html).
