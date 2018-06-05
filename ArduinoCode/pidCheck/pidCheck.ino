/*
 * Author : Kunchala Anil
 * Email : anilkunchalaece@gmail.com
 * Date : 05 Jun 2018
 * 
 * Simple PID implementation
 */

const float pitchPVal=0.0;
const float pitchDVal=0.0;
const float pitchIVal=0.0;
const float rollPVal=0.0;
const float rollDVal=0.0;
const float rollIVal=0.0;



float pidPitchIn,pidRollIn,pidYawIn=0;
float pidPitchOut,pidRollOut,pidYawOut=0;

float pitchError,rollError,yawError=0;
float pitchPrevError,rollPrevError,yawPrevError = 0;
float pitchErrorSum,rollErrorSum,yawErrorSum = 0;//for integral coefficient
float pitchErrorDelta,rollErrorDelta,yawErrorDelta = 0;//for Derivative coefficients

float compAngleY,compAngleX;
float setPointY,setPointX;
 
void setup() {
  
}//end of setup Fcn

void loop() {

}//end of loop Fcn

void pidCalculation(){
  //calculate errors
  pitchError = compAngleY - setPointY;
  rollError = compAngleX - setPointX;

  //calculate sum of errors
  pitchErrorSum = pitchError + pitchPrevError;
  rollErrorSum = rollError + rollPrevError;

  //calculate delta of errors
  pitchErrorDelta = pitchError - pitchPrevError;
  rollErrorDelta = rollError - rollPrevError;

  //save current error as prev error for next iteration
  pitchPrevError = pitchError;
  rollPrevError = rollError;

  //PID calculation - taken from - github.com/lobodol/drone-flightcontroller
  pidPitchOut = (pitchError * pitchPVal) + (pitchErrorSum * pitchIVal) + (pitchErrorDelta * pitchDVal);
  pidRollOut = (rollError * rollPVal) + (rollErrorSum * rollIVal) + (rollErrorDelta * rollIVal); 
}//end of pidCalculation Fcn

