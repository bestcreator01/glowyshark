/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815
  
  These drivers use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
int analogPin0 = A0; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
int analogPin1 = A1;
int analogPin2 = A2;
int val = 0;  // variable to store the value read
int val1 = 0;
int val2 = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

// You can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. It's not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVO_FREQ;   // Analog servos run at ~60 Hz updates
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000000;  // convert input seconds to us
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

/*
  There are three rotational servos installed beneath the shark model, enabling it to move in a singular, 
  forward direction along a circular track. The movement of the shark is defined by three distinct scenarios:

  1) Dark Movement: This scenario serves as the default state. In this mode, the shark remains stationary but engages in subtle, 
                    wavy movements to simulate the appearance of floating underwater.
  2) Bright Movement: Triggered by the detection of an approaching individual via a photosensor, this scenario activates the light sensor, 
                      simulating the shark's instinct to swim. The shark moves away swiftly, covering half the distance of the track to seek 
                      refuge behind an object, during which lights are turned on to enhance the effect.
  3) General Movement: Following its escape, the shark returns to its original position, completing a circuit around the track to simulate 
                      cautiously coming out of hiding. Lights are turned off as the shark resumes its default state. 
                      Once it reaches the starting point, the cycle begins anew, starting with the Dark Movement scenario.

  These scenarios collectively aim to mimic the natural behavior of a shark in various states of rest, alertness, and movement, providing a dynamic and interactive experience.
*/

/*
  This method works as the general movement
*/
void transitionMovement() {
  // Simulate swimming back to the starting position with more pronounced movement
  for (uint16_t i = 1250; i < 1800; i += 100) {
    pwm.writeMicroseconds(0, i); // Front servo, initiating turn
    pwm.writeMicroseconds(1, 1750); // Middle servo, less movement
    delay(500); // Adjust for desired speed of movement
  }
  // Optional: Add logic to adjust for a precise stop at the default position
}


/*
  This method works as the bright movement
*/
void brightMovement() {
  // Simulate a quick start to escape
  int startPulse = 1750; // Starting pulse for rapid movement
  int endPulse = 1250; // End pulse, simulating slowing down to hide
  int step = (startPulse - endPulse) / 5; // Divide the movement into steps for smooth transition

  // Accelerate
  for (int pulse = startPulse; pulse >= endPulse; pulse -= step) {
    pwm.writeMicroseconds(0, pulse); // Adjust servo for forward movement
    delay(200); // Short delay for quick start
  }

  // Optional: Hold the position for a moment to simulate hiding
  pwm.writeMicroseconds(0, endPulse);
  delay(1000); // Wait for 1 second to simulate the shark hiding

  // The shark remains in this hiding position until the transitionMovement() is called
}


/*
  This method works as the dark movement
*/
void darkMovement() {
  // Gentle oscillation around a central position
  for (int angle = 1450; angle <= 1650; angle += 100) {
    pwm.writeMicroseconds(0, angle); // Front servo
    pwm.writeMicroseconds(1, 2000 - angle); // Middle servo, opposite phase
    pwm.writeMicroseconds(2, angle); // Tail servo, same phase as front
    delay(600); // Adjust for a smooth, slow movement
  }
  for (int angle = 1650; angle >= 1450; angle -= 100) {
    pwm.writeMicroseconds(0, angle);
    pwm.writeMicroseconds(1, 2000 - angle); // Middle servo, opposite phase
    pwm.writeMicroseconds(2, angle); // Tail servo, same phase as front
    delay(600); // Adjust for a smooth, slow movement
  }
}

void loop() {
  val = analogRead(analogPin0); // Assume this reads ambient light or proximity
  if (val > 600) { // Bright, someone is close
    brightMovement();
    delay(10000); // Wait before returning
    transitionMovement();
  } else {
    darkMovement();
  }
}


// void loop() {

//   val = analogRead(analogPin0);
//   val1 = analogRead(analogPin1);
//   val2 = analogRead(analogPin2);
//   int sum = (val + val1 + val2) / 3;

//   // When it is bright
//   if (sum > 600)
//   {
//     uint16_t i = 1250;
//     uint16_t j = 1750;
//     while (i < 1800 && j > 1200)
//     {
//       pwm.writeMicroseconds(2, i);
//       pwm.writeMicroseconds(0, j);
//       delay(1000);
//       i += 250;
//       j -= 250;
//     }
//   }
//   else
//   {
    
//   }
//   // Drive each servo one at a time using setPWM()
//   // Serial.println(servonum);
//   // for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
//   //   pwm.setPWM(servonum, 0, pulselen);
//   // }

//   // delay(500);
//   // for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
//   //   pwm.setPWM(servonum, 0, pulselen);
//   // }

//   // delay(500);

//   // // Drive each servo one at a time using writeMicroseconds(), it's not precise due to calculation rounding!
//   // // The writeMicroseconds() function is used to mimic the Arduino Servo library writeMicroseconds() behavior. 
//   // for (uint16_t microsec = USMIN; microsec < USMAX; microsec++) {
//   //   pwm.writeMicroseconds(servonum, microsec);
//   // }
//   uint16_t i = 1250;
//   uint16_t j = 1750;
//   while (i < 1800 && j > 1200)
//   {
//     pwm.writeMicroseconds(1, i);
//     pwm.writeMicroseconds(0, j);
//     delay(1000);
//     i += 250;
//     j -= 250;
//   }
//   pwm.sleep();

//   // delay(500);
//   // for (uint16_t microsec = USMAX; microsec > USMIN; microsec--) {
//   //   pwm.writeMicroseconds(servonum, microsec);
//   // }

//   // delay(500);

//   // servonum++;
//   // if (servonum > 7) servonum = 0; // Testing the first 8 servo channels
// }
