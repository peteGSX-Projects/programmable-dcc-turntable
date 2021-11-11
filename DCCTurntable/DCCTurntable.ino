/*************************************************************
DCC Turntable controller.

This uses DCC commands to control a ULN2003 stepper motor controller
and 28BYJ-48 stepper motor to drive a turntable.

This also uses the AccelStepper library to neatly control the turntable bridge
neatly including acceleration/deceleration.

See the README for the full list of features and instructions.
*************************************************************/

#include <AccelStepper.h>
#include <NmraDcc.h>

// Define our pins etc.
#define DCC_PIN 2                           // Pin to receive DCC signal
const uint8_t DccAckPin = A1;               // Pin connected to the DCC ACK circuit
uint16_t baseTurntableAddress;              // First turntable position address
#define HOME_SENSOR_PIN 3                   // Pin connected to the home sensor
#define HOME_SENSOR_ACTIVE_STATE LOW        // State to flag when home
const int maxTurntablePositions = 10;       // This much match the array elements defined later
bool lastIsRunningState;                    // Store last running state to help disable stepper

// Define decoder version
#define DCC_DECODER_VERSION_NUM 1

// Turn stepper off when not actually running
#define DISABLE_OUTPUTS_IDLE

// The lines below define the pins used to connect to the ULN2003 driver module
const uint8_t uln2003Pin1 = 8;
const uint8_t uln2003Pin2 = 9;
const uint8_t uln2003Pin3 = 10;
const uint8_t uln2003Pin4 = 11;
const uint8_t uln2003Step = 4;        // Tells the driver to use all 4 pins for a full step

// By default the stepper motor will move the shortest distance to the desired position.
// If you need the turntable to only move in the Positive/Increasing or Negative/Decreasing step numbers to better handle backlash in the mechanism
// Then uncomment the appropriate line below
//#define ALWAYS_MOVE_POSITIVE
//#define ALWAYS_MOVE_NEGATIVE

// The lines below define the stepping speed and acceleration, which you may need to tune for your application
#define STEPPER_MAX_SPEED     800   // Sets the maximum permitted speed
#define STEPPER_ACCELARATION  50  // Sets the acceleration/deceleration rate
#define STEPPER_SPEED         200   // Sets the desired constant speed for use with runSpeed()

// Define number of steps per rotation and per half rotation
const uint16_t fullTurnSteps = 2048;

// This constant is useful to know the number of steps to rotate the turntable 180 degrees for the back entrance position
const uint16_t halfTurnSteps = fullTurnSteps / 2;

// This structure holds the values for a turntable position with the DCC Address, Front Position in Steps from Home Sensor
typedef struct
{
  uint16_t dccAddress;
  uint16_t positionFront;
  uint16_t positionBack;
}
turntablePosition;
turntablePosition turntablePositions[maxTurntablePositions];

// --------------------------------------------------------------------------------------------
// You shouldn't need to edit anything below this line unless you're needing to make big changes... ;)
// --------------------------------------------------------------------------------------------
#if defined(ALWAYS_MOVE_POSITIVE) && defined(ALWAYS_MOVE_NEGATIVE)
#error ONLY uncomment one of ALWAYS_MOVE_POSITIVE or ALWAYS_MOVE_NEGATIVE but NOT both
#endif

// Setup the AccelStepper object for the ULN2003 Stepper Motor Driver
AccelStepper stepper1(uln2003Step, uln2003Pin1, uln2003Pin3, uln2003Pin2, uln2003Pin4);

// Dcc Accessory Decoder object
NmraDcc  Dcc ;
DCC_MSG  Packet;

// Variables to store the last DCC Turnout message Address and Direction  
uint16_t lastAddr = 0xFFFF ;
uint8_t lastDirection = 0xFF;
int     lastStep = 0;

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

// Function to perform a software reset
void(* resetFunc) (void) = 0;

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
  digitalWrite( DccAckPin, HIGH );
  delay( 10 );  
  digitalWrite( DccAckPin, LOW );
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
  Serial.print(F("notifyDccAccTurnoutOutput: "));
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;
  for (int i = 0; i < maxTurntablePositions ; i++)
  {
    if ((Addr == turntablePositions[i].dccAddress) && ((Addr != lastAddr) || (Direction != lastDirection)) && OutputPower)
    {
      lastAddr = Addr ;
      lastDirection = Direction ;
      Serial.print("Position front/back is: ");
      Serial.print(turntablePositions[i].positionFront);
      Serial.print("/");
      Serial.println(turntablePositions[i].positionBack);
      Serial.print(F("Moving to "));
      Serial.print(Direction ? F("Front") : F("Back"));
      Serial.print(F(" Position: "));
      Serial.print(i, DEC);
      Serial.print(F(" @ Step: "));
      int newStep;
      if(Direction) {
        newStep = turntablePositions[i].positionFront;
      } else {
        newStep = turntablePositions[i].positionBack;
      }
      Serial.print(F("  Move: "));
      Serial.println(newStep, DEC);
      /* Change/revise logic below from moveTo to move, calculate steps in code
       *  Move in whichever direction makes the move closer is the goal
       *  Current code has motor moving anti-clockwise to the specified step position
       */
      stepper1.moveTo(newStep);
      /*
      Serial.print(newStep, DEC);
      Serial.print(F("  Last Step: "));
      Serial.print(lastStep, DEC);
      int diffStep = newStep - lastStep;
      Serial.print(F("  Diff Step: "));
      Serial.print(diffStep, DEC);
      if(diffStep > halfTurnSteps) {
        diffStep = diffStep - fullTurnSteps;
      } else if(diffStep < -halfTurnSteps) {
        diffStep = diffStep + fullTurnSteps;
      }
      Serial.print(F("  Move: "));
      Serial.println(diffStep, DEC);
      stepper1.move(diffStep);
      lastStep = newStep;
      */
      break;
    }
  }
};

void initPositions() {
  // This array contains the Turnout Positions which can have lines added/removed to suit your turntable 
  turntablePositions[0] = (turntablePosition) {baseTurntableAddress + 0, 0, 0 + halfTurnSteps };
  turntablePositions[1] = (turntablePosition) {baseTurntableAddress + 1, 150, 150 + halfTurnSteps };
  turntablePositions[2] = (turntablePosition) {baseTurntableAddress + 2, 300, 300 + halfTurnSteps };
  turntablePositions[3] = (turntablePosition) {baseTurntableAddress + 3, 450, 450 + halfTurnSteps };
  turntablePositions[4] = (turntablePosition) {baseTurntableAddress + 4, 600, 600 + halfTurnSteps };
  turntablePositions[5] = (turntablePosition) {baseTurntableAddress + 5, 750, 750 + halfTurnSteps };
  turntablePositions[6] = (turntablePosition) {baseTurntableAddress + 6, 900, 900 + halfTurnSteps };
  turntablePositions[7] = (turntablePosition) {baseTurntableAddress + 7, 1050, 1050 + halfTurnSteps };
  turntablePositions[8] = (turntablePosition) {baseTurntableAddress + 8, 1200, 1200 + halfTurnSteps };
  turntablePositions[9] = (turntablePosition) {baseTurntableAddress + 9, 2000, 2000 + halfTurnSteps };
}

void setupStepperDriver() {
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED);        // Sets the maximum permitted speed
  stepper1.setAcceleration(STEPPER_ACCELARATION); // Sets the acceleration/deceleration rate
  stepper1.setSpeed(STEPPER_SPEED);               // Sets the desired constant speed for use with runSpeed()
}

bool moveToHomePosition() {
  Serial.println(F("Finding Home Sensor...."));
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
  Serial.println("Setting up DCC Decorder...");
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  // Many Arduino Cores now support the digitalPinToInterrupt() function that makes it easier to figure out the
  // Interrupt Number for the Arduino Pin number, which reduces confusion. 
#ifdef digitalPinToInterrupt
  Dcc.pin(DCC_PIN, 0);
#else
  Dcc.pin(0, DCC_PIN, 1);
#endif
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );
}

uint16_t getBaseAddress() {
  // Function to retrieve the base accessory address from the EEPROM and validate it
  // Retrieve the current values from the EEPROM
  uint16_t cvMSB = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB);
  uint16_t cvLSB = Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB);
  uint16_t eepromBaseTurntableAddress = (((cvMSB * 64) + cvLSB - 1) * 4) + 1  ;
  // Validate our MSB and CSB values are valid, otherwise use default decoder address of 1
  if ((cvMSB == 0 && cvLSB == 0) || cvMSB > 7 || cvLSB > 63) {
    Serial.println("WARNING: The EEPROM stored CVs contain invalid MSB and/or LSB values, returning default of 1");
    Serial.print("MSB value: ");
    Serial.print(cvMSB);
    Serial.print("LSB value: ");
    Serial.println(cvLSB);
    eepromBaseTurntableAddress = 1;
  // Validate that this returns an actual valid DCC Decoder accessory base address here
  } else if (eepromBaseTurntableAddress + maxTurntablePositions >= 2041) {
    Serial.println("WARNING: The EEPROM stored CVs contain an address that would exceed the upper valid address, returning default of 1");
    Serial.print("Upper valid address would be: ");
    Serial.println(eepromBaseTurntableAddress + maxTurntablePositions);
    eepromBaseTurntableAddress = 1;
  }
  return eepromBaseTurntableAddress;
}

void setup() {
  Serial.begin(115200);
  while(!Serial);   // Wait for the USB Device to Enumerate
  baseTurntableAddress = getBaseAddress();
  Serial.println((String)"NMRA DCC Turntable Controller version " + DCC_DECODER_VERSION_NUM);
  Serial.print("Full Rotation Steps: ");
  Serial.println(fullTurnSteps);
  Serial.print("Movement Strategy: ");
#if defined ALWAYS_MOVE_POSITIVE
  Serial.println("Positive Direction Only");
#elif defined ALWAYS_MOVE_NEGATIVE
  Serial.println("Negative Direction Only");
#else
  Serial.println("Shortest Distance");
#endif
  initPositions();
  Serial.print("Base turntable DCC address: ");
  Serial.println(baseTurntableAddress, DEC);
  for(uint8_t i = 0; i < maxTurntablePositions; i++)
  {
    Serial.print("DCC Addr: ");
    Serial.print(turntablePositions[i].dccAddress);

    Serial.print(" Front: ");
    Serial.print(turntablePositions[i].positionFront);

    Serial.print(" Back: ");
    Serial.println(turntablePositions[i].positionBack);
  }
  
  setupStepperDriver();
  if(moveToHomePosition()) { 
    setupDCCDecoder();

    // Fake a DCC Packet to cause the Turntable to move to Position 1
    notifyDccAccTurnoutOutput(baseTurntableAddress, 1, 1);
  }
}

void loop() {
  while (Serial.available() > 0) {
    int moveToPosition = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.println((String)"Moving to position " + moveToPosition);
      notifyDccAccTurnoutOutput(moveToPosition, 1, 1);
    }
  }
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

  // Process the Stepper Library
  stepper1.run();

#ifdef DISABLE_OUTPUTS_IDLE
  if(stepper1.isRunning() != lastIsRunningState)
  {
    lastIsRunningState = stepper1.isRunning();
    if(!lastIsRunningState)
    {
      stepper1.disableOutputs();
      Serial.println("Disable Stepper Outputs");
    }
  }
#endif
  if ( FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
    resetFunc();    // If we've had a factory reset performed, reset the Arduino to start with the new addressing.
  }
}
