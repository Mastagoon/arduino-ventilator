/*  ####################### 
 *  code by Sahal Alzubair 
 *  Sudanese Ventilator 20
 *  WARNING: this is still a work in progress.
 *  ####################### 
 *
 *  #TODO: the speed value does not take into account the pause delay.
 *  code can be more simplified
 *  ARDUINO MEGA VERSION
*/
#include <Wire.h>
#include <Servo.h>
#include <sfm3000wedo.h>
/* pins */ 
const int servoPin = 13;
const int speedIncBtn = 21; // speed value is controlled by changing the delay inside the servo-write loops
const int speedDecBtn = 20;
const int volIncBtn = 19; // volume value is controlled by changing the target angle
const int volDecBtn = 18;

//objects
Servo servo1; // create servo object
SFM3000wedo measflow(64); //sensor flow object

//sensors stuff
int offset = 32000; // Offset for the flow sensor #TODO: revision this
float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8 #TODO: revision this

/* speed variables */
int currentSpeed = 16; // 16 times per minute, half-cycle every 1.875s
const int speedInc = 1; // increase cycles by 1 everytime the button is pressed
const int speedDec = 1; // decrease by 1
const int maxSpeed = 30; // maximum possible value for speed
const int minSpeed = 10; // minimum possible value for speed
const int inhaleToExhaleRatio = 0.66; // inhaling takes 33% of the total process time (1:2 ratio), ex if time is 1s, inhale in 0.33s and exhale in 0.67s
const int pauseDelay = 1500; // the pause delay between inhaling and exhaling

/* angle variables */
int targetAngle = 130; // angle goes from 0 to this value and vise-versa #TODO change this to start and end values for more flexability
const int angleInc = 10; // increase the angle value by this amount 
const int angleDec = 10; // decrease the angle value by this amount #TODO use only 1 variable for this (needs more testing)
const int maxAngle = 180; // maximum possible angle for the servo motor
const int minAngle = 40; // minimum possible angle for the servo motor

int getDelay(int speedVal,bool inhale) { // this function takes the speed value and returns the needed delay
  if(inhale) { // first half cycle.
    int steps = targetAngle - minAngle; // get the number of angle steps the servo must take
    return ((60000/speedVal*inhaleToExhaleRatio)/steps); // the delay value for inhaling (note that the return statement ends the function here.)
    /* for debugging */
    //Serial.println(60000/speedVal*inhaleToExhaleRatio);
  }
  int steps = targetAngle - minAngle;
  return ((60000/speedVal/(1-inhaleToExhaleRatio))/steps); // the delay value for exhaling (if inhale = false)
}

bool safetyCheck() { // checks for and corrects conditions that could cause harm or malfunction.
  if(targetAngle > maxAngle) { // checks if the maximum allowed angle is exceeded.
    targetAngle = maxAngle;
    Serial.print("warning: max angle exceeded, angle is reset to ");
    Serial.println(maxAngle);
  }
  if(targetAngle < minAngle) {
    targetAngle = minAngle;
    Serial.print("warning: target Angle value is below min angle, angle is reset to ");
    Serial.println(minAngle);
  }
  if(currentSpeed > maxSpeed) { // checks if the maximum speed value is exceeded.
    currentSpeed = maxSpeed;
    Serial.print("warning: speed excceded the maximum allowed limit, speed is reset to ");
    Serial.println(maxSpeed);
  }
  if(currentSpeed < minSpeed) {
    currentSpeed = minSpeed;
    Serial.print("warning: speed is below the minimum allowed limit, speed is reset to ");
    Serial.println(minSpeed);
  }
  return true;
}

void increaseSpeed() {
    delay(50);
    currentSpeed += speedInc;
    Serial.print("Speed increased, current speed value is ");
    Serial.println(currentSpeed);
}

void decreaseSpeed() {
    delay(50);
    currentSpeed -= speedDec;
    Serial.print("Speed decreased, current speed value is ");
    Serial.println(currentSpeed);
}

void increaseVolume() {
    delay(50);
    targetAngle += angleInc;
    Serial.print("Target angle increased, current angle value is ");
    Serial.println(targetAngle);
}

void decreaseVolume() {
    delay(50);
    targetAngle -= angleDec;
    Serial.print("Target angle decreased, current angle value is ");
    Serial.println(targetAngle);
}

void setup() {
  pinMode(speedIncBtn, INPUT);
  pinMode(speedDecBtn, INPUT);
  pinMode(volIncBtn, INPUT);
  pinMode(volDecBtn, INPUT);
  attachInterrupt(digitalPinToInterrupt(speedIncBtn), increaseSpeed, RISING);
  attachInterrupt(digitalPinToInterrupt(speedDecBtn), decreaseSpeed, RISING);
  attachInterrupt(digitalPinToInterrupt(volIncBtn), increaseVolume, RISING);
  attachInterrupt(digitalPinToInterrupt(volDecBtn), decreaseVolume, RISING);
  servo1.attach(servoPin);
  servo1.write(minAngle);
  Wire.begin();
  Serial.begin(9600);
  delay(500); // delay to set the servo and startup the serial console
  measflow.init();
  Serial.println("Flow sensor initilized.");
}

void loop() {
  if(safetyCheck()) { // if the device passes the safety check, start working immediately
    unsigned int flowSensorResult = measflow.getvalue(); // get the reading from the flow sensor
    float flow = ((float)result - offset) / scale; // some math magic to get the actual flow value
    // a calculation to get the tidal volume value from the flow goes here.
    Serial.print("The flow value is ");
    Serial.println(flow);
    Serial.print("target angle : ");
    Serial.println(targetAngle);
    Serial.print("current speed : ");
    Serial.println(currentSpeed);
    for(int i = minAngle; i <= targetAngle; i++) { // move the servo from 0 to the target angle
      servo1.write(i);
      delay(getDelay(currentSpeed, true)); // get the needed delay to keep the process at the desired current speed
    }
    delay(pauseDelay); // delay between inhaling and exhaling
    for(int i = targetAngle; i >= minAngle; i--) { // move the servo from the current target angle to 0
      servo1.write(i);
      delay(getDelay(currentSpeed, false)); // get the needed delay for the return trip speed
    }
  }
}
