#include "SimpleRSLK.h"
uint16_t value;
uint16_t sensorVal[LS_NUM_SENSORS];
uint16_t sensorCalVal[LS_NUM_SENSORS];
uint16_t sensorMaxVal[LS_NUM_SENSORS];
uint16_t sensorMinVal[LS_NUM_SENSORS];
int cntPerRevolution = 360;
float Pi = 3.14;
float wheelDiameter = 7; /*in centimeters*/
bool isCalibrationComplete = false;
unsigned long timeBegin;
unsigned long timeEnd;

void setup(){
  Serial.begin(9600);
  setupRSLK();
  /* Left button on Launchpad */
  setupWaitBtn(LP_RIGHT_BTN);
  /* Red led in rgb led */
  setupLed(GREEN_LED);
  setupLed(RED_LED);
  clearMinMax(sensorMinVal,sensorMaxVal);
}

void loop() {
  waitBtnPressed(LP_RIGHT_BTN);
  if(isCalibrationComplete == false) {
     simpleCalibrate(); 
     isCalibrationComplete = true;
  }
  waitBtnPressed(LP_RIGHT_BTN);
  blink();
  linefollowing();
}

void blink(){
  for(int i=0;i<3;i++){
  digitalWrite(GREEN_LED, HIGH);   /* turn the LED on (HIGH is the voltage level)*/
  delay(500);               /* wait for a second*/
  digitalWrite(GREEN_LED, LOW);    /* turn the LED off by making the voltage LOW*/
  delay(500); 
  }
}

float distanceTraveled(float wheel_diam, float cnt_per_rev, float current_cnt) {
  float temp = (wheel_diam *Pi* current_cnt)/cnt_per_rev;
  return temp;
}

void printing(){
  unsigned long dist=distanceTraveled(7,360,getEncoderRightCnt());
  Serial.println("LAB Group #15");
  Serial.println("The robot travelled..."); 
  Serial.print(dist); //distance
  Serial.println("centimeters");
  Serial.print(dist/100);
  Serial.println("meters");
  Serial.print(dist*0.393);
  Serial.println("inches");
  Serial.print(dist*0.032);
  Serial.println("feet\n");
  
  Serial.println("It took...");//time and speed
  unsigned long timer=((timeEnd-timeBegin)/1000000);
  Serial.print(timer);
  Serial.println("seconds");
  Serial.println("at avg speed of..."); 
  Serial.print(dist/timer);
  Serial.println("centimeters/second");
  Serial.println("\n\n");
  delay(5000);
}

void simpleCalibrate() {
  /* Set both motors direction forward */
  setMotorDirection(BOTH_MOTORS,MOTOR_DIR_FORWARD);
  /* Enable both motors */
  enableMotor(BOTH_MOTORS);
  /* Set both motors speed 20 */
  setMotorSpeed(BOTH_MOTORS,20);

  for(int x = 0;x<50;x++){
    readLineSensor(sensorVal);
    setSensorMinMax(sensorVal,sensorMinVal,sensorMaxVal);
  }

  /* Disable both motors */
  disableMotor(BOTH_MOTORS);
}

void linefollowing(){
 enableMotor(BOTH_MOTORS); 
 float normalSpeed = 23.19; /* ratio = 3.449 */
 uint16_t fastSpeed = 80;
  
 float normalSpeed2 = 35;   /* ratio = 4.5714 */
 uint16_t fastSpeed2 = 160;
  
 float normalSpeed3 = 10;   /* ratio = 12 */
 uint16_t fastSpeed3 = 120;

  
 float straightSpeed = 40.5;
  
 resetRightEncoderCnt();//resetting right encoder
  
 uint8_t lineColor = DARK_LINE;/*DARK_LINE  if your floor is lighter than your line*/
 timeBegin = micros();//strt timr
 while(true){
  readLineSensor(sensorVal);
  readCalLineSensor(sensorVal,sensorCalVal,sensorMinVal,sensorMaxVal,lineColor);
  
  value=0;
  
  uint32_t linePos = getLinePosition(sensorCalVal,lineColor); 
  
  for (uint8_t i = 0; i < LS_NUM_SENSORS; i++){ 
   value+=sensorVal[i];
  }
  if(value>19000){ //stopping condition
    timeEnd = micros();//stop timr
    disableMotor(RIGHT_MOTOR);
    disableMotor(LEFT_MOTOR);
    digitalWrite(RED_LED, HIGH);
    break;
 }
  if(linePos > 0 && linePos < 1200){  /*extreme left turn*/
   setRawMotorSpeed(LEFT_MOTOR,normalSpeed3);
   setRawMotorSpeed(RIGHT_MOTOR,fastSpeed3);
}
   else if(linePos > 5800 && linePos < 7000) { /*extreme right turn*/
    setRawMotorSpeed(LEFT_MOTOR,fastSpeed3);
    setRawMotorSpeed(RIGHT_MOTOR,normalSpeed3);
  } 
   else if(linePos > 1200 && linePos < 2200) {  /*2nd last left turn*/
    setRawMotorSpeed(LEFT_MOTOR,normalSpeed2);
    setRawMotorSpeed(RIGHT_MOTOR,fastSpeed2);
  } 
  else if(linePos > 4800 && linePos < 5800) {  /*2nd last extreme right turn*/
    setRawMotorSpeed(LEFT_MOTOR,fastSpeed2);
    setRawMotorSpeed(RIGHT_MOTOR,normalSpeed2);
  }
  else if(linePos > 3800 && linePos < 4800) {  /*3rd last extreme right turn*/
    setRawMotorSpeed(LEFT_MOTOR,fastSpeed);
    setRawMotorSpeed(RIGHT_MOTOR,normalSpeed);
  }
  else if(linePos > 2200 && linePos < 3200) { /* 3rd last  left turn*/
    //5000 > linePos > 3500) 
    setRawMotorSpeed(LEFT_MOTOR,normalSpeed);
    setRawMotorSpeed(RIGHT_MOTOR,fastSpeed);
} 
  else { 
    setRawMotorSpeed(LEFT_MOTOR,straightSpeed);
    setRawMotorSpeed(RIGHT_MOTOR,straightSpeed);
}
}
while(true)printing();
}
