#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 3
#define ENC_IN_RIGHT_A 2

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 6
#define ENC_IN_RIGHT_B 11

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16MultiArray encoder_ticks;
ros::Publisher array_pub("encoder_ticks", &encoder_ticks);

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;

// Array to store wheel tick counts
int wheel_ticks[2] = {0, 0}; // Initialize with two elements, both set to 0

// Increment the number of ticks for the right wheel
void right_wheel_tick() {
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {   
    if (wheel_ticks[1] == encoder_maximum) {
      wheel_ticks[1] = encoder_minimum;
    }
    else {
      wheel_ticks[1]++;
    }
  }
  else {
    if (wheel_ticks[1] == encoder_minimum) {
      wheel_ticks[1] = encoder_maximum;
    }
    else {
      wheel_ticks[1]--;
    }
  }
}

// Increment the number of ticks for the left wheel
void left_wheel_tick() {
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (wheel_ticks[0] == encoder_maximum) {
      wheel_ticks[0] = encoder_minimum;
    }
    else {
      wheel_ticks[0]++;
    }
  }
  else {
    if (wheel_ticks[0] == encoder_minimum) {
      wheel_ticks[0] = encoder_maximum;
    }
    else {
      wheel_ticks[0]--;
    }
  }
}

void setup() {
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
 
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(array_pub);
  wheel_ticks[0] = 0;
  wheel_ticks[1] = 0;
}

void loop() {
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, publish the ticks
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
     
    encoder_ticks.data_length = 2;
    encoder_ticks.data = wheel_ticks;
    
    array_pub.publish(&encoder_ticks);
    
    nh.spinOnce();
  }
}
