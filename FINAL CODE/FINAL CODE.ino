// Last updated by: Mark DeLoura
// UPDATED: 1/12/2022

#include <Stepper.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TRSim_Raven.h>


/* Raven Launch demo
     This simple demo reads data coming from the Future Engineers TechRise web-based simulator.
     It is designed to run on a Metro M4 Express and uses the on-board Neopixel.
     The demo runs a simple update loop, and does three actions:
       1 - Keeps track of the number of telemetry packets received
       2 - Monitors for new events and prints a message each time one occurs
       3 - Prints out every 1000th telemetry packet
*/
int trig = 3; // digital pin 4
const int stepsPerRevolution = 1024;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

// pin mappings
//   Note: this pin already defined in header files
// #define PIN_NEOPIXEL     40
#define PIN_PBF 2
#define PIN_GO  3

// neopixel constants
#define NEOPIXEL_QUANTITY 1
#define NEOPIXEL_BRIGHTNESS 0.2
#define IDLE_COLOR 0x808080

// Set up events
#define EVENTS_QUANTITY 4
#define EVENT_LAND 0
#define EVENT_RISING 1
#define EVENT_FLOATING 2
#define EVENT_DESCENDING 3

// Event threshold values to determine RISE, FLOAT, DESCEND
#define RISE_THRESHOLD -2.0
#define FLOAT_THRESHOLD 4.0
#define DESCEND_THRESHOLD 5.0

typedef struct _eventDetails {
  int value;
  const char* description;
  uint32_t color;
} EventDetails;
EventDetails events[] = {
  {EVENT_LAND, "Liftoff", 0xff0000}, // red
  {EVENT_RISING, "We are rising", 0x00ff00}, // green
  {EVENT_FLOATING, "We are floating", 0x00ffff}, // cyan
  {EVENT_DESCENDING, "We are descending", 0x0000ff} // blue
};


// Set up Neopixels
Adafruit_NeoPixel pixels(NEOPIXEL_QUANTITY, PIN_NEOPIXEL);

// Set up Simulator
TRSim_Raven::Simulator TRsim;

// Baud rate for USB serial connection
#define SERIAL_BAUD 115200

// Variables for tracking events
int currEvent = EVENT_LAND;
int prevEvent = currEvent;
// Variable for tracking number of full telemetry packets received
int numPackets = 0;

float currVelocityDown;

unsigned char* data;

char outputString[200];

void serialHandleNewEvent(int e);
void serialPrintPacket();
void serialPrintHexString(unsigned char* buff);

// setup()
//   Initialization functions
//
void setup() {

  // initialize the digital pins as output.
  pinMode(trig, OUTPUT);

  digitalWrite(trig, HIGH);
  // set the speed at 60 rpm:
  myStepper.setSpeed(20);
  // initialize the serial port:
  // Serial = USB
  // TRSim = Serial1 = UART
  Serial.begin(SERIAL_BAUD);
  delay(2000); // Improves chances of seeing first printed messages over USB serial
  Serial.println("Running Raven Launch demo");

  // Initialize simulator
  TRsim.init(PIN_PBF, PIN_GO);

  // Print out the starting state of the PBF header
  if (TRsim.getPBF() == HIGH) {
    Serial.println("Pull-before flight header removed");
  } else {
    Serial.println("Pull-before flight header inserted");
  }
  Serial.println();

  // Flash the GO led a few times
  // go_pin will be set according to the state of the pbf_pin when TRsim.update() is called
  for (int i = 0; i < 5; i++) {
    TRsim.setGo((TRsim.getGo() == HIGH) ? LOW : HIGH);
    delay(500);
  }

  // Neopixels
  pixels.begin();
  pixels.setBrightness(255 * NEOPIXEL_BRIGHTNESS);
  pixels.fill(0x777777, 0, NEOPIXEL_QUANTITY);
  pixels.show();
}

// loop()
//   Do forever
//
void loop() {
  // Update the simulator to catch serial input
  TRsim.update();

  // If there is a new full telemetry packet, do some operations on it
  if (TRsim.isStreaming() == true) {
    if (TRsim.isNewData() == true) {
      // Got a new telemetry packet!
      numPackets += 1;

      // Grab new data - NOTE this sets isNewData to false!
      data = TRsim.getData();

      // Grab the velocityD property from the new data packet
      currVelocityDown = TRsim.getVelocityDown();

      // This is a crude way to use the data to estimate the state of the balloon
      //   The event values flutter a little at launch/liftoff and state changes
      if (currVelocityDown < RISE_THRESHOLD) {
        currEvent = EVENT_RISING;
      } else if ((currVelocityDown >= RISE_THRESHOLD) &&
                 (currVelocityDown < DESCEND_THRESHOLD) &&
                 (currEvent != EVENT_LAND)) {
        currEvent = EVENT_FLOATING;
      } else if (currVelocityDown >= DESCEND_THRESHOLD) {
        currEvent = EVENT_DESCENDING;
      } else {
        currEvent = EVENT_LAND;
      }

      // now that we have a current event we can compare it to the previous
      if (currEvent != prevEvent) {
        // Save current event as previous for next time
        prevEvent = currEvent;
        // A new event has occurred, let's do something with it

        // Handle the new event by printing a message or changing LED color
        serialHandleNewEvent(currEvent);
      }

      // Print every 100th packet to verify data
      if ((numPackets % 100) == 1) {
        serialPrintPacket();
      }
    }
  } else { // not streaming
    // the data stream has stopped for 1.5s, fill the pixels white
    pixels.fill(IDLE_COLOR, 0, NEOPIXEL_QUANTITY);
    pixels.show();
    // and reset the current event
    prevEvent = EVENT_LAND;
  }

  delay(10);
}

// serialHandleNewEvent(e)
//   Print out new event and alter Neopixel color if appropriate
//
void serialHandleNewEvent(int e) {
  // check if the PBF header is open
  if (TRsim.getPBF() == HIGH) {
    // indicate the new event with a color from the event dictionary
    pixels.fill(events[e].color, 0, NEOPIXEL_QUANTITY);
  } else {
    // don't light the pixels becuase we're not flying yet!
    pixels.fill(0, 0, NEOPIXEL_QUANTITY);
  }
  pixels.show();

  // since the event changed print something to indicate
  Serial.println(events[e].description);
}

// serialPrintPacket()
//   Prints out contents of a data packet to the USB serial connection
//
void serialPrintPacket() {
  serialPrintHexString(data);
  if (TRsim.getAltitude() > 15000) {
    Serial.println(TRsim.getAltitude());
    Serial.println("Altitude reached!!");
    for (int i = 0; i < 7; i++) {

      digitalWrite(trig, LOW);

      delay(50);

      digitalWrite(trig, HIGH);

      //Delay between pictures
      delay(500);

      // step one revolution  in one direction:
      Serial.println("clockwise");
      myStepper.step(stepsPerRevolution);


      delay(15000);

      digitalWrite(trig, LOW);

      delay(150);

      digitalWrite(trig, HIGH);

      //Delay between pictures
      delay(500);

      delay(15000);


      // step one revolution in the other direction:
      Serial.println("counterclockwise");
      myStepper.step(-stepsPerRevolution);
      delay(6000);
    }
  }

  sprintf(outputString, "  time %d.%06d", TRsim.getTimeSecs(), TRsim.getTimeUsecs());
  Serial.println(outputString);
  Serial.print("  altitude ");
  Serial.println(TRsim.getAltitude());

}

// serialPrintHexString(buff)
//
void serialPrintHexString(unsigned char* buff) {
  int cursor = 0;

  for (int i = 0; i < 79; i++) {
    sprintf(&(outputString[cursor]), "%02x", buff[i]);
    cursor += 2;
  }
  outputString[cursor] = 0;

  Serial.println(outputString);
}
