/*************************************************************
DCC Turntable controller.

This uses DCC commands to control a ULN2003 stepper motor controller
and 28BYJ-48 stepper motor to drive a turntable.

This also uses the AccelStepper library to neatly control the turntable bridge
neatly including acceleration/deceleration.

See the README for the full list of features and instructions.
*************************************************************/

// Include our libraries
#include <AccelStepper.h>
#include <NmraDcc.h>

// Create our DCC objects
NmraDcc  Dcc;
DCC_MSG  Packet;

// Define our global variables
#define DCC_PIN     2                           // DCC input interupt pin
const int positionDegrees = 12;                 // Degree separation between positions
const int DccAckPin = A1;                       // DCC ACK output pin
uint16_t BaseTurnoutAddress;                    // First turnout address
int positionsInUse[] = {1,2,3,4,10,20,30};      // Array defining the positions actually in use
//int positionsInUse[0];                          // If all positions are in use, leave array empty

// Set up our AccelStepper object
#define FULLSTEP 4
AccelStepper myStepper(FULLSTEP,8,10,9,11);

// Calculate the available positions based on the degree of separation, must be evenly divisible!
const int positionsAvailable = 360 / positionDegrees;
long stepsPerPosition = 2048 / positionsAvailable;

// Configure a struct for our positions then calculate step positions from 0 for each position to populate it
typedef struct {
  int positionIndex;                            // Index of the position
  long steps;                                   // Steps from 0 to reach the position
} position_def;

position_def positions[positionsAvailable];

// Define decoder version
#define DCC_DECODER_VERSION_NUM 1

// Define the struct for CVs
struct CVPair {
  uint16_t  CV;
  uint8_t   Value;
};

// Define the factory default address CV pair (key, value)
CVPair FactoryDefaultCVs [] = {
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, DEFAULT_ACCESSORY_DECODER_ADDRESS & 0xFF},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, DEFAULT_ACCESSORY_DECODER_ADDRESS >> 8},
};

// Define the index in the array that holds the factory default CVs
uint8_t FactoryDefaultCVIndex = 0;

void notifyCVResetFactoryDefault() {
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void) {
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
  for(uint8_t i = 0; i < Msg->Size; i++) {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower ) {
  Serial.print("notifyDccAccTurnoutOutput: ");
  Serial.print(Addr,DEC);
  Serial.print(',');
  Serial.print(Direction,DEC);
  Serial.print(',');
  Serial.println(OutputPower, HEX);
  // If the address of the DCC turnout is one of these, act on it
  if(( Addr >= BaseTurnoutAddress ) && ( Addr < (BaseTurnoutAddress + positionsAvailable )) && OutputPower ) {
    uint8_t position = Addr - BaseTurnoutAddress;    // Calculate which array reference to use
    Serial.println((String)"Move turntable to position " + position);
    /*
    unsigned long currentDccMillis = millis();
    // Flag the change if our turnout's new and current position match, and if we've exceeded the delay to the next switching time
    if (points[point].currentDirection == points[point].newDirection && currentDccMillis - points[point].lastSwitchEndMillis > switchingDelay) {
      // Only proceed if the new direction is different to the current direction
      if ((points[point].currentDirection == LOW && Direction == 1) || (points[point].currentDirection == HIGH && Direction == 0)) {
        // If new direction is 0 (close) but our current direction is HIGH (throw), flag the change and record the time for pulsing
        if (Direction == 0 && points[point].currentDirection == HIGH) {
          points[point].newDirection = LOW;
        // If new direction is 1 (throw) but our current direction is LOW (close), flag the change and record the time for pulsing
        } else if (Direction == 1 && points[point].currentDirection == LOW) {
          points[point].newDirection = HIGH;
        }
        Serial.println((String)"Setting turnout at address " + Addr + " (point " + point + ") to " + Direction);
        // Perform our pin digital write depending on motor config
        #ifdef FUNDUMOTO
          digitalWrite(points[point].directionPin, points[point].newDirection);
        #elif defined(L293D)
          if (points[point].newDirection == LOW) {
            digitalWrite(points[point].directionPin1, HIGH);
            digitalWrite(points[point].directionPin2, LOW);
          } else if (points[point].newDirection == HIGH) {
            digitalWrite(points[point].directionPin1, LOW);
            digitalWrite(points[point].directionPin2, HIGH);
        #endif
        // Set speed to start switching and record when it started so we can pulse correctly
        analogWrite(points[point].speedPin, 255);
        points[point].lastSwitchStartMillis = currentDccMillis;
      }
    }
    */
  }
}

void initTurntable() {
  // Function to initialise the turntable control pins etc.
  for (int i = 0; i < positionsAvailable; i++) {
    positions[i] = (position_def){i, stepsPerPosition};
  }
}

void setup() {
  Serial.begin(115200);
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );
  Serial.println((String)"NMRA DCC Rokuhan Turnout Controller version " + DCC_DECODER_VERSION_NUM);
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
  BaseTurnoutAddress = (((Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_MSB) * 64) + Dcc.getCV(CV_ACCESSORY_DECODER_ADDRESS_LSB) - 1) * 4) + 1  ;
  // Initialise our turnouts
  initTurntable();
  uint8_t lastPosition = positionsAvailable - 1;
  Serial.print((String)"Init Done, base turntable address is: " + BaseTurnoutAddress + " and last turntable address is ");
  Serial.println (BaseTurnoutAddress + lastPosition, DEC);
  // Set up our stepper motor speeds
  myStepper.setMaxSpeed(1000.0);
  myStepper.setAcceleration(50.0);
  myStepper.setSpeed(200);
}

void loop() {
  while (Serial.available() > 0) {
    int moveToPosition = Serial.parseInt();
    if (Serial.read() == '\n') {
      Serial.println((String)"Moving to position " + moveToPosition);
      //myStepper.moveTo(steps);
    }
  }
  //myStepper.run();
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
