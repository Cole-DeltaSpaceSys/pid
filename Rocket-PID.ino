/* Delta Space Systems
 * Rocket-PID v0.1
*/


/* Tuning Guide - README
 *  To tune the servo offsets for your Actuator go to line 52. 
    To tune just raise the values until the actuator switches directions then home in on the perfect value.
 *  For PID Value Changes go to line 112 - 114.
 *  To change what I/O pins the actuators are connected to go to line 139 - 140.  
 */


//PID Variables
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY;


//Upright Angle of the Gyroscope
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

//Offsets for tuning 
int servoY_offset = -87;
int servoX_offset = 144;

//Position of servos through the startup function
int servoXstart = servoY_offset * -1;
int servoYstart = servoX_offset;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 7;
float servoY_gear_ratio = 9;

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;


//PID Gains
double kp = 0.11;
double ki = 0.02;
double kd = 0.04;

double dt, currentTime, previousTime;
 

void setup(){
  
}
void loop() {
  //Defining Time Variables      
  currentTime = millis();            
  dt = (currentTime - previousTime) / 1000; 
  
}

void pidcompute () {
previous_errorX = errorX;
previous_errorY = errorY; 

//Inputs to the PID Controller
errorX = /*AngleX*/ - desired_angleX;
errorY = /*AngleY*/ - desired_angleY;

//Defining "P" 
pidX_p = kp*errorX;
pidY_p = kp*errorY;

//Defining "D"
pidX_d = kd*((errorX - previous_errorX)/dt);
pidY_d = kd*((errorY - previous_errorY)/dt);

//Defining "I"
pidX_i = ki * (pidX_i + errorX * dt);
pidY_i = ki * (pidY_i + errorY * dt);

//Adding it all up
PIDX = pidX_p + pidX_i + pidX_d;
PIDY = pidY_p + pidY_i + pidY_d;

//Outputs to send to a actuator
pwmY = ((PIDY * servoY_gear_ratio) + servoX_offset);
pwmX = -1*((PIDX * servoX_gear_ratio) + servoY_offset); 

}
