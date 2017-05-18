
"""
#include <RPLidar.h>
// You need to create an driver instance 
RPLidar lidar;

#define RPLIDAR_MOTOR 9 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
                        
#define PT_ARRAY_SIZE 35
#define SAFETY_DISTANCE 1500 //mm
#define TIME_DETECT 1000 //ms
#define ANGLE_SCOPE 22.5
#define FORWARD_ANGLE 270
#define RED_PIN  2
#define GREEN_PIN 3
#define BLUE_PIN 4


bool scanning = true;
String inputString = "";
boolean stringComplete = false;

bool newPoint = false;
float ptArray[PT_ARRAY_SIZE];
int ptArray_counter = 0;
int collision_counter = 0;
int current_time = 0;
int time_last_detect = 0;

void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial2);
  Serial.begin(9600);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  delay(2000);
  analogWrite(RPLIDAR_MOTOR, 255);
  for(int i = 0; i < PT_ARRAY_SIZE; i++){
    ptArray[i] = SAFETY_DISTANCE * 1.5;
  }
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}
void loop() {
   scanLidar();
 
   if(newPoint){
     float avg_dist = 0;
     for(int i = 0; i < PT_ARRAY_SIZE; i++){
       avg_dist += ptArray[i];
     }
     avg_dist /= PT_ARRAY_SIZE;
     Serial.println(avg_dist);
     if(avg_dist < SAFETY_DISTANCE){
      if(current_time - time_last_detect > TIME_DETECT)
        collision_counter = 0;
      time_last_detect = current_time;
      collision_counter ++;
      
      setColor(255,0,0);
      
      if(collision_counter > 50){
        analogWrite(RPLIDAR_MOTOR, 0);
        for(int i = 255; i >= 0; i-=25){
        setColor(0, i, 255);
        delay(200);
        }
        delay(2000);
        analogWrite(RPLIDAR_MOTOR, 255);
        collision_counter = 0;
        setColor(0,0,0);
      }
     }
     else{
      
      setColor(0,0,0);
     }
     newPoint = false;
   }
   */
}

void scanLidar(){
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance;
    float angle    = lidar.getCurrentPoint().angle;
    int quality = lidar.getCurrentPoint().quality;
    if(quality < 10) return;
    current_time = millis();
    Serial.print(current_time);
    Serial.print(",");
    Serial.print(distance);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(quality);
    Serial.print("\n");
    
    if(quality > 5 && angle < FORWARD_ANGLE + ANGLE_SCOPE && angle > FORWARD_ANGLE - ANGLE_SCOPE){
      ptArray[ptArray_counter] = distance;
      ptArray_counter ++;
      newPoint = true;
      if(ptArray_counter == PT_ARRAY_SIZE){
        ptArray_counter = 0;
      }
    }
    
  } else {
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       lidar.startScan();
       delay(1000);
    }
    
  }
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
  red = 255 - red;
  green = 255 - green;
  blue = 255 - blue;
  #endif
  analogWrite(RED_PIN, red);
  analogWrite(GREEN_PIN, green);
  analogWrite(BLUE_PIN, blue);
}
"""