// Auguste Brown

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// assign constants
Servo ESC;
int ESC_PIN = 9;
int PWM_MIN = 1000;
int PWM_MAX = 2000;
const float delta_t = 25; //ms
const int total_t = 10; //sec
const int num_samples = 1000 * total_t / delta_t;
//float max_thrust = 2 * 9.81;

float theta;
float angular_v = 0;
float sum = 0;
int i = 0;




// converts input number from degrees into radians
float degrees_to_radians(float degrees) {
    return degrees * (2 * 3.14159 / 360);
}

int controller() {
    float Kp = 2.5;
    float Kd = 0.6;
    float Ki = 0.18;

    sum += theta;
    
    int output = (Kp * -theta) + (Kd * -angular_v) + (Ki * -sum);
    output = constrain(output, 0, 400);

    return output + PWM_MIN;
}

void setup() {
    Serial.begin(9600);
    ESC.attach(ESC_PIN, PWM_MIN, PWM_MAX); 
    ESC.writeMicroseconds(PWM_MIN);
    
    /* Initialise the sensor */
    if(!bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    
    delay(1000);
      
    bno.setExtCrystalUse(true);
    delay(1000);
}

sensors_event_t event;

void loop() {
    if (i >= 0 and i < num_samples){
      bno.getEvent(&event);

      float old_theta = theta;
      theta = -event.orientation.y;

      if (i > 0){
        angular_v = (theta - old_theta) / (delta_t / 1000);
      }
      else {
        angular_v = 0;
      }

      int motor_output = controller();

      ESC.writeMicroseconds(motor_output);

      
      Serial.print(theta);
      Serial.print(",");
      Serial.print(angular_v);
      Serial.print(",");
      Serial.println(motor_output);
    }
    else {
      ESC.writeMicroseconds(PWM_MIN);
    }
    
    i++;
    delay(delta_t);
}
