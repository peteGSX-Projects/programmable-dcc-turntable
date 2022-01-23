/*************************************************************
DCC Turntable controller.

This uses DCC commands to control a ULN2003 stepper motor controller
and 28BYJ-48 stepper motor to drive a turntable as well as a dual relay board
to reverse the polarity of the bridge track as required.

Turntable positions are programmed via DCC commands as per a normal accessory decoder.

For more information and instructions refer to the README in the GitHub repo:
https://github.com/peteGSX/programmable-dcc-turntable

This also uses the AccelStepper library to neatly control the turntable bridge
neatly including acceleration/deceleration.

Much credit to Alex Shepherd for the example DCC turntable sketch provided with
the NrmaDcc library.

See the README for the full list of features and instructions.
*************************************************************/

#include <AccelStepper.h>
#include <NmraDcc.h>
#include <avr/wdt.h>

// Define our pins
#define DCC_PIN 2                           // Pin to receive DCC signal
#define DCC_ACK_PIN A1                      // Pin connected to the DCC ACK circuit
#define HOME_SENSOR_PIN 3                   // Pin connected to the home sensor
#define ULN2003_PIN1 8                      // ULN stepper controller pin 1
#define ULN2003_PIN2 9                      // ULN stepper controller pin 2
#define ULN2003_PIN3 10                     // ULN stepper controller pin 3
#define ULN2003_PIN4 11                     // ULN stepper controller pin 4
#define RELAY1 4                            // Control pin for relay 1
#define RELAY2 5                            // Control pin for relay 2

// Define global variables
uint16_t baseTurntableAddress;              // First turntable position address
#define HOME_SENSOR_ACTIVE_STATE LOW        // State to flag when home
//uint16_t numTurntablePositions = 1;         // Placeholder assignment of 1 until retrieved from CV
//const int maxTurntablePositions = 50;       // Define a sane limit of positions
bool lastIsRunningState;                    // Store last running state to help disable stepper
const uint16_t numPositionsCV = 513;        // CV number to store the number of turntable positions in
int16_t lastStep = 0;                       // Record of the last step position moved to
const int16_t fullTurnSteps = 2048;         // Define steps for a full turn
const int16_t halfTurnSteps = fullTurnSteps / 2; // Define steps for a half turn, used to move least distance
uint16_t lastAddr = 0xFFFF;                 // Record of the last DCC addressed received

// Define decoder version
#define DCC_DECODER_VERSION_NUM 1

// Turn stepper off when not actually running
#define DISABLE_OUTPUTS_IDLE

// The lines below define the stepping speed and acceleration, which you may need to tune for your application
#define STEPPER_MAX_SPEED     100   // Sets the maximum permitted speed
#define STEPPER_ACCELARATION  25  // Sets the acceleration/deceleration rate
#define STEPPER_SPEED         100   // Sets the desired constant speed for use with runSpeed()

// Setup the AccelStepper object for the ULN2003 Stepper Motor Driver
//AccelStepper stepper1(AccelStepper::FULL4WIRE, ULN2003_PIN1, ULN2003_PIN3, ULN2003_PIN2, ULN2003_PIN4); // Counter clockwise
AccelStepper stepper1(AccelStepper::FULL4WIRE, ULN2003_PIN4, ULN2003_PIN2, ULN2003_PIN3, ULN2003_PIN1);   // Clockwise

// Dcc Accessory Decoder object
NmraDcc  Dcc ;
DCC_MSG  Packet;

// Define the struct for CVs
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// Define the factory default address CV pair (key, value)
CVPair FactoryDefaultCVs [] =
{
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8},
};

// Define the index in the array that holds the factory default CVs
uint8_t FactoryDefaultCVIndex = 0;

void resetFunc() {
  // Function to perform a software reset courtesy of the DCC++ EX team
  wdt_enable (WDTO_15MS);       // set Arduino watchdog timer for 15ms 
  delay(50);                    // wait for the prescaler time to expire          
}

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
  Serial.println("notifyCVAck");
  digitalWrite( DCC_ACK_PIN, HIGH );
  delay( 10 );  
  digitalWrite( DCC_ACK_PIN, LOW );
}

// Uncomment to print all DCC Packets
//#define NOTIFY_DCC_MSG
#ifdef  NOTIFY_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg) {
  Serial.print("notifyDccMsg: ");
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  if ((Addr == baseTurntableAddress || (Addr > baseTurntableAddress && Addr < baseTurntableAddress + Dcc.getCV(numPositionsCV))) && OutputPower && stepper1.isRunning() == false && Addr != lastAddr) {
    lastAddr = Addr;
    uint8_t cvOffset = Addr - baseTurntableAddress;
    uint16_t stepsLSBCV = numPositionsCV + (cvOffset * 3) + 1;
    uint8_t stepsLSB = Dcc.getCV(stepsLSBCV);
    uint16_t stepsMSBCV = numPositionsCV + (cvOffset * 3) + 2;
    uint8_t stepsMSB = Dcc.getCV(stepsMSBCV);
    uint16_t polarityCV = numPositionsCV + (cvOffset * 3) + 3;
    uint8_t polarity = Dcc.getCV(polarityCV);
    int16_t steps = (stepsMSB << 8) + stepsLSB;
    if (steps <= fullTurnSteps && polarity < 2) {
      int16_t moveSteps;
      Serial.print((String)"Notification to " + Addr + ": ");
      Serial.print((String)"Position steps: " + steps + ", Polarity flag: " + polarity);
      if ((steps - lastStep) > halfTurnSteps) {
        moveSteps = steps - fullTurnSteps - lastStep;
      } else if ((steps - lastStep) < -halfTurnSteps) {
        moveSteps = fullTurnSteps - lastStep + steps;
      } else {
        moveSteps = steps - lastStep;
      }
      Serial.println((String)" - moving " + moveSteps + " steps");
      setPolarity(polarity);
      lastStep = steps;
      stepper1.move(moveSteps);
    } else {
      Serial.println((String)"ERROR: CV definitions for " + Addr + " are invalid, not moving");
    }
  }
};

void printPositions() {
  Serial.println((String)Dcc.getCV(numPositionsCV) + " turntable positions defined:");
  for (uint8_t i = 0; i < Dcc.getCV(numPositionsCV); i++) {
    Serial.print("DCC addr ");
    Serial.print(baseTurntableAddress + i, DEC);
    Serial.print(" definition: ");
    uint16_t stepsLSBCV = numPositionsCV + (i * 3) + 1;
    uint8_t stepsLSB = Dcc.getCV(stepsLSBCV);
    uint16_t stepsMSBCV = numPositionsCV + (i * 3) + 2;
    uint8_t stepsMSB = Dcc.getCV(stepsMSBCV);
    uint16_t polarityCV = numPositionsCV + (i * 3) + 3;
    uint8_t polarity = Dcc.getCV(polarityCV);
    uint16_t steps = (stepsMSB << 8) + stepsLSB;
    if (steps > fullTurnSteps) {
      Serial.println((String)"ERROR! Steps cannot exceed " + fullTurnSteps);
    }
    if (polarity > 1) {
      Serial.println("ERROR! Polarity must be 0 or 1");
    }
    Serial.print((String)steps + " steps (LSB CV " + stepsLSBCV + "=" + stepsLSB);
    Serial.print((String)", MSB CV " + stepsMSBCV + "=" + stepsMSB);
    Serial.println((String)") with polarity flag " + polarity + " (CV " + polarityCV + ")");
  }
}

void setupStepperDriver() {
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);        // Sets the maximum permitted speed
  stepper1.setAcceleration(STEPPER_ACCELARATION); // Sets the acceleration/deceleration rate
  stepper1.setSpeed(STEPPER_SPEED);               // Sets the desired constant speed for use with runSpeed()
}

bool moveToHomePosition() {
  pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
  stepper1.move(fullTurnSteps * 2);
  while(digitalRead(HOME_SENSOR_PIN) != HOME_SENSOR_ACTIVE_STATE)
    stepper1.run();
  if(digitalRead(HOME_SENSOR_PIN) == HOME_SENSOR_ACTIVE_STATE) {
    stepper1.stop();
    stepper1.setCurrentPosition(0);
    Serial.println(F("Found Home Position - Setting Current Position to 0"));
    return true;
  } else {
    Serial.println(F("Home Position NOT FOUND - Check Sensor Hardware"));
  }
  return false;  
}

void setupDCCDecoder() {
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, DCC_DECODER_VERSION_NUM, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 550);
}

uint16_t getBaseAddress() {
  // Function to retrieve the base accessory address from the EEPROM and validate it
  // Retrieve the current values from the EEPROM
  uint16_t cvMSB = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB);
  uint16_t cvLSB = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB);
  uint16_t eepromBaseTurntableAddress = (((cvMSB * 64) + cvLSB - 1) * 4) + 1  ;
  //uint16_t numTurntablePositions = Dcc.getCV(numPositionsCV);
  // Validate our MSB and CSB values are valid, otherwise use default decoder address of 1
  if ((cvMSB == 0 && cvLSB == 0) || cvMSB > 7 || cvLSB > 63) {
    Serial.println("WARNING: The EEPROM stored address CVs contain invalid MSB and/or LSB values, returning default of 1");
    Serial.print("MSB value: ");
    Serial.print(cvMSB);
    Serial.print(" LSB value: ");
    Serial.println(cvLSB);
    eepromBaseTurntableAddress = 1;
  // Validate that this returns an actual valid DCC Decoder accessory base address here
  } else if (eepromBaseTurntableAddress + Dcc.getCV(numPositionsCV) >= 2041) {
    Serial.println("WARNING: The EEPROM stored CVs contain an address that would exceed the upper valid address, returning default of 1");
    Serial.print("Upper valid address would be: ");
    Serial.println(eepromBaseTurntableAddress + Dcc.getCV(numPositionsCV));
    eepromBaseTurntableAddress = 1;
  }
  return eepromBaseTurntableAddress;
}

void setPolarity(uint8_t Polarity) {
  // Function to set the correct polarity for the bridge track, 0 normal, 1 reverse
  digitalWrite(RELAY1, Polarity);
  digitalWrite(RELAY2, Polarity);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);   // Wait for the USB Device to Enumerate
  Serial.println((String)"Programmable DCC Turntable Controller version " + DCC_DECODER_VERSION_NUM);
  Serial.println((String)"Full Rotation Steps: " + fullTurnSteps);
  baseTurntableAddress = getBaseAddress();  // Get our base DCC address
  //initPositions();  // Initialise our array of positions
  pinMode(RELAY1, OUTPUT);  // Set our relay pins to output
  pinMode(RELAY2, OUTPUT);
  printPositions();
  setupStepperDriver(); // Initialise the stepper driver
  if(moveToHomePosition()) {
    setupDCCDecoder();
    // Fake a DCC Packet to cause the Turntable to move to Position 1
    notifyDccAccTurnoutOutput(baseTurntableAddress, 1, 1);
    #ifdef DISABLE_OUTPUTS_IDLE
      stepper1.disableOutputs();
    #endif
  }
}

void loop() {
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  // Process the Stepper Library
  stepper1.run();
  // If we've enabled it, disable stepper motor when not actively turning
#ifdef DISABLE_OUTPUTS_IDLE
  if(stepper1.isRunning() != lastIsRunningState)
  {
    lastIsRunningState = stepper1.isRunning();
    if(!lastIsRunningState)
    {
      stepper1.disableOutputs();
    }
  }
#endif
  // If we flagged a DCC reset, do it
  if ( FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
    resetFunc();    // If we've had a factory reset performed, reset the Arduino to start with the new addressing.
  }
}
