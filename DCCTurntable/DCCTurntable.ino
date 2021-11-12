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
const int maxTurntablePositions = 12;       // This much match the array elements defined later
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

// The lines below define the stepping speed and acceleration, which you may need to tune for your application
#define STEPPER_MAX_SPEED     400   // Sets the maximum permitted speed
#define STEPPER_ACCELARATION  50  // Sets the acceleration/deceleration rate
#define STEPPER_SPEED         200   // Sets the desired constant speed for use with runSpeed()

// Define number of steps per rotation and per half rotation
const int16_t fullTurnSteps = 2048;

// This constant is useful to know the number of steps to rotate the turntable 180 degrees for the back entrance position
const int16_t halfTurnSteps = fullTurnSteps / 2;

// This structure holds the values for a turntable position with the DCC Address, Front Position in Steps from Home Sensor
typedef struct
{
  uint16_t dccAddress;
  uint16_t positionFront;
}
turntablePosition;
turntablePosition turntablePositions[maxTurntablePositions];

// Setup the AccelStepper object for the ULN2003 Stepper Motor Driver
AccelStepper stepper1(uln2003Step, uln2003Pin1, uln2003Pin3, uln2003Pin2, uln2003Pin4);

// Dcc Accessory Decoder object
NmraDcc  Dcc ;
DCC_MSG  Packet;

// Variables to store the last DCC Turnout message Address and Direction  
uint16_t lastAddr = 0xFFFF ;
int     lastStep = 0;
uint8_t lastPosition = 0;

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
  for (uint8_t i = 0; i < maxTurntablePositions ; i++)
  {
    if ((Addr == turntablePositions[i].dccAddress) && (Addr != lastAddr) && OutputPower) {
      Serial.print("Position is: ");
      Serial.println(turntablePositions[i].positionFront);
      Serial.print(F("Moving to "));
      Serial.print(Direction ? F("Front") : F("Back"));
      Serial.print(F(" Position: "));
      Serial.println(i, DEC);
      int newStep = turntablePositions[i].positionFront;
      int lastStep = turntablePositions[lastPosition].positionFront;
      Serial.print("newStep: ");
      Serial.print(newStep);
      Serial.print(" lastStep: ");
      Serial.println(lastStep);
      int moveStep;
      Serial.print("Moving ");
      // If moving to our new position is more than half a turn, go anti-clockwise
      if ((newStep - lastStep) > halfTurnSteps) {
        moveStep = newStep - fullTurnSteps - lastStep;
      } else if ((newStep - lastStep) < -halfTurnSteps) {
        moveStep = fullTurnSteps - lastStep + newStep;
      } else {
        moveStep = newStep - lastStep;
      }
      Serial.print(moveStep, DEC);
      Serial.println(" steps");
      lastPosition = i;
      stepper1.move(moveStep);
      break;
    }
  }
};

void initPositions() {
  // This array contains the Turnout Positions which can have lines added/removed to suit your turntable 
  turntablePositions[0] = (turntablePosition) {baseTurntableAddress + 0, 0};
  turntablePositions[1] = (turntablePosition) {baseTurntableAddress + 1, 150};
  turntablePositions[2] = (turntablePosition) {baseTurntableAddress + 2, 300};
  turntablePositions[3] = (turntablePosition) {baseTurntableAddress + 3, 450};
  turntablePositions[4] = (turntablePosition) {baseTurntableAddress + 4, 600};
  turntablePositions[5] = (turntablePosition) {baseTurntableAddress + 5, 750};
  turntablePositions[6] = (turntablePosition) {baseTurntableAddress + 6, 900};
  turntablePositions[7] = (turntablePosition) {baseTurntableAddress + 7, 1050};
  turntablePositions[8] = (turntablePosition) {baseTurntableAddress + 8, 1200};
  turntablePositions[9] = (turntablePosition) {baseTurntableAddress + 9, 1500};
  turntablePositions[10] = (turntablePosition) {baseTurntableAddress + 10, 1800};
  turntablePositions[11] = (turntablePosition) {baseTurntableAddress + 11, 2000};
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
  Dcc.init( MAN_ID_DIY, DCC_DECODER_VERSION_NUM, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );
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
    Serial.print(" LSB value: ");
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
  initPositions();
  Serial.print("Base turntable DCC address: ");
  Serial.println(baseTurntableAddress, DEC);
  for(uint8_t i = 0; i < maxTurntablePositions; i++)
  {
    Serial.print("DCC Addr: ");
    Serial.print(turntablePositions[i].dccAddress);

    Serial.print(" Front: ");
    Serial.println(turntablePositions[i].positionFront);
  }
  setupStepperDriver();
  if(moveToHomePosition()) { 
    setupDCCDecoder();
    // Fake a DCC Packet to cause the Turntable to move to Position 1
    notifyDccAccTurnoutOutput(baseTurntableAddress, 1, 1);
    stepper1.disableOutputs();
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
