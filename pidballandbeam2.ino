// including libraries to control distance sensor
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 


// create sensor object to control the distance sensor
SFEVL53L1X distanceSensor;

// PID values that can be tuned
double Kp = 10;
double Ki = 2;
double Kd = 255;


 
unsigned long current_time, last_time;

double elapsed_time;

double Error;

double previous_error;

double input, output, Setpoint;

double cum_error, rate_error;


void setup(void)
{
  Wire.begin();
  Serial.begin(115200);
  // Setpoint to where the ball has to end up (in mm)
  Setpoint = 270;
  
  
}

void loop(void)
{
  // Initializing the sensor to start measureing distance
  distanceSensor.startRanging(); 


  // Getting the result of the measurement from the sensor
  int distance = distanceSensor.getDistance(); 
  
  // Getting the measurements set as our imput for the PID
  input = distance; 

  // Printing the measurements in serial monitor
  Serial.println(input);

  
  output = computePID(input);
  delay(100);

  // Setting the servo to be the output, which will output the computed PID value
  analogWrite(9, output); 
  
}
 
    double computePID(double inp){   
        
        current_time = millis();                //Current time is made with the millis command
        
        elapsed_time = (double)(current_time - last_time);        //code for previous time elapsed made
        
        Error = Setpoint - inp;                                // Code for detecting the error
        
        cum_error += Error * elapsed_time;                // Calculate the integral part
        
        rate_error = (Error - previous_error)/elapsed_time;   // calculation for the derivative part
 
        double out = Kp*Error + Ki*cum_error + Kd*rate_error;                //outputs for the PID             
 
       
        previous_error = Error;                                //The current error has to be remembered as the previous
        
        last_time = current_time;                        //Same with the current error - has to remember the curren time
 
        return out;         
}
