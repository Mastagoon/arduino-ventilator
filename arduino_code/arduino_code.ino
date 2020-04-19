/*  ####################### 
/*  code by Sahal Alzubair 
 *  Sudanese Ventilator 20
 *  WARNING: this is still a work in progress.
 *  ####################### 
 */
 /*  #TODO: the speed value does not take into account the pause delay.
  *  code can be more simplified
 */
#include <Servo.h>
/* pins */ 
const int servoPin = 13;
const int speedIncBtn = 12; // speed value is controlled by changing the delay inside the servo-write loops
const int speedDecBtn = 11;
const int volIncBtn = 10; // volume value is controlled by changing the target angle
const int volDecBtn = 9;
Servo servo1; // create servo object

/* speed variables */
int currentSpeed = 16; // 16 times per minute, half-cycle every 1.875s
const int speedInc = 1; // increase cycles by 1 everytime the button is pressed
const int speedDec = 1; // decrease by 1
const int maxSpeed = 20; // maximum possible value for speed
const int minSpeed = 10; // minimum possible value for speed
const int inhaleToExhaleRatio = 0.33; // inhaling takes 33% of the total process time (1:2 ratio), ex if time is 1s, inhale in 0.33s and exhale in 0.67s
const int pauseDelay = 1500; // the pause delay between inhaling and exhaling

/* angle variables */
int targetAngle = 130; // angle goes from 0 to this value and vise-versa #TODO change this to start and end values for more flexability
const int angleInc = 10; // increase the angle value by this amount 
const int angleDec = 10; // decrease the angle value by this amount #TODO use only 1 variable for this (needs more testing)
const int maxAngle = 130; // maximum possible angle for the servo motor
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

void checkButtons() {
  int speedIncBtnVal = digitalRead(speedIncBtn); // reads if the speed increment button is pressed
  speedIncBtnVal ? currentSpeed += speedInc : currentSpeed = currentSpeed; // if speedIncBtnVal is high, increase speed, else do nothing
  int speedDecBtnVal = digitalRead(speedDecBtn); // the same code is repeated for all 4 buttons
  speedDecBtnVal ? currentSpeed -= speedDec : currentSpeed = currentSpeed;
  int AngleIncBtnVal = digitalRead(volIncBtn);
  AngleIncBtnVal ? targetAngle += angleInc : targetAngle = targetAngle;
  int AngleDecBtnVal = digitalRead(volDecBtn);
  AngleDecBtnVal ? targetAngle -= angleDec : targetAngle = targetAngle;
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

void setup() {
  pinMode(speedIncBtn, INPUT);
  pinMode(speedDecBtn, INPUT);
  pinMode(volIncBtn, INPUT);
  pinMode(volDecBtn, INPUT);
  servo1.attach(servoPin);
  servo1.write(0);
  Serial.begin(9600);
}

void loop() {
  checkButtons(); // checks values of buttons first to detect any change
  if(safetyCheck()) { // if the device passes the safety check, start working immediately
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
