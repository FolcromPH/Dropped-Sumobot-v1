#include "NewPing.h"

// define IR 
#define ir_right A1
#define ir_left A2

// Define motor pins
#define IN1A 3
#define IN1B 5
#define IN2A 6
#define IN2B 9

//define ultrasonic
#define LTRIGGER_PIN 11
#define LECHO_PIN 12
#define RTRIGGER_PIN 7
#define RECHO_PIN 8
#define MAX_DISTANCE 50
#define MIN_DISTANCE 2
NewPing sonarL(LTRIGGER_PIN, LECHO_PIN, MAX_DISTANCE);
NewPing sonarR(RTRIGGER_PIN, RECHO_PIN, MAX_DISTANCE);


// Global variables
int Irr_value;
int Irf_value;
float distanceL;
float distanceR;

// Calibration values
int left_black_threshold = 640; // Adjust as needed
int left_white_threshold = 610; // Adjust as needed
int right_black_threshold = 720; // Adjust as needed
int right_white_threshold = 10; // Adjust as needed

void setup() {
  // Set motor & enable connections as outputs
  pinMode(IN1A, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(IN2B, OUTPUT);

  // Stop motors
  stop();
  delay(5000);
  Serial.begin(9600);
}

void loop() {
   // Read IR sensor values
  Irr_value = analogRead(ir_right);
  Irf_value = analogRead(ir_left);


  if(ColorChecker()){
    ultraSensor();
  }else{
    Serial.print("WHITE");
    stop();
    reverse();
    delay(500);
  }

  delay(100); // Delay to avoid flooding the serial port
}

void ultraSensor() {
  int iterations = 1;

  // Get sensor data
  float durationL = sonarL.ping_median(iterations);
  float durationR = sonarR.ping_median(iterations);

  // Determine distance from duration
  distanceL = (durationL / 2) * 0.0343;
  distanceR = (durationR / 2) * 0.0343;


  //check left
  if (distanceL <= MAX_DISTANCE && distanceL >= MIN_DISTANCE) {
    Serial.println(distanceL);
    Serial.println("attackL");
    forward();
        //delay(1000);

  }
  else if (distanceR <= MAX_DISTANCE && distanceR >= MIN_DISTANCE) {
    Serial.println(distanceR);
    Serial.println("attackR");
    forward();
       // delay(1000);

  } else {
    Serial.println("spin");
    left();
  }
  
}

bool ColorChecker() {
  if (Irr_value > right_black_threshold || Irf_value > left_black_threshold) {
    return true; // Both sensors see black
  } else {
    return false; // At least one sensor sees white
  }
}

void forward() {
  digitalWrite(IN1A, LOW);
  digitalWrite(IN1B, LOW);
  analogWrite(IN2A, 200);
  analogWrite(IN2B, 200);
}

void reverse() {
  digitalWrite(IN2A, LOW);
  digitalWrite(IN2B, LOW);
  analogWrite(IN1A, 200);
  analogWrite(IN1B, 200);
}

void right() {
  digitalWrite(IN1A, LOW);
  digitalWrite(IN2B, LOW);
  analogWrite(IN1B, 200);
  analogWrite(IN2A, 200);
}

void left() {
  digitalWrite(IN1B, LOW);
  digitalWrite(IN2A, LOW);
  analogWrite(IN1A, 200);
  analogWrite(IN2B, 200);
}

void stop() {
  analogWrite(IN1A, 0);
  analogWrite(IN1B, 0);
  analogWrite(IN2A, 0);
  analogWrite(IN2B, 0);
}
