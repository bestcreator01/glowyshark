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

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver();

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

int countRotation = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  pwm2.begin();
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

  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

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
  pwm2.setPWM(n, 0, pulse);
}

/*
  There are three rotational servos installed beneath the shark model, enabling it to move in a singular, 
  forward direction along a circular track. The movement of the shark is defined by three distinct scenarios:

  1) Dark Movement: This scenario serves as the default state. In this mode, the shark remains stationary but engages in subtle, 
                    wavy movements to simulate the appearance of floating underwater.
  2) Bright Movement: Triggered by the detection of an approaching individual via photoresistors, this scenario activates the light sensor, 
                      simulating the shark's instinct to swim. The shark moves away swiftly, covering half the distance of the track to seek 
                      refuge behind an object, during which lights are turned on to enhance the effect.
  3) General Movement: Following its escape, the shark returns to its original position, completing a circuit around the track to simulate 
                      cautiously coming out of hiding. Lights are turned off as the shark resumes its default state. 
                      Once it reaches the starting point, the cycle begins anew, starting with the Dark Movement scenario.

  These scenarios collectively aim to mimic the natural behavior of a shark in various states of rest, alertness, and movement, providing a dynamic and interactive experience.
*/

/*
  This helper method would help the shark to move around the track moving 180 degrees.
*/
void move180Degrees()
{
  pwm2.writeMicroseconds(2, 1600);
  delay(800);

  // Tail wiggling
  pwm.writeMicroseconds(1, 1470);
  delay(300);
  pwm.writeMicroseconds(1, 1850);
  delay(300);

  pwm2.writeMicroseconds(2, 0);
}

void moveBackwards()
{
  pwm2.writeMicroseconds(2, 1390);
  delay(600);

  // Tail wiggling
  pwm.writeMicroseconds(1, 1470);
  delay(300);
  pwm.writeMicroseconds(1, 1850);
  delay(300);

  pwm2.writeMicroseconds(2, 0);
}

void wiggle()
{
  // Gentle oscillation around a central position
  pwm.writeMicroseconds(0, 1470);
  pwm.writeMicroseconds(1, 1470);
  delay(500); 
  pwm.writeMicroseconds(0, 1850);
  pwm.writeMicroseconds(1, 1850);
  delay(500);
}

void loop() {
  val = analogRead(analogPin0); // Assume this reads ambient light or proximity
  val1 = analogRead(analogPin1);
  val2 = analogRead(analogPin2);

  int val3 = (val + val1 + val2) / 3;

  if (val3 > 500) { // Bright, someone is close
    move180Degrees();

    while(val3 > 500)
    {
      Serial.println("Waiting for dark");
      val = analogRead(analogPin0);
      val1 = analogRead(analogPin1);
      val2 = analogRead(analogPin2); 

      val3 = (val + val1+ val2) / 3;
    }
    moveBackwards();
  } 
  else {
    wiggle();
  }
}