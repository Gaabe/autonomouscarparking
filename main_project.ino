#include <NewPing.h>
#include <Servo.h>
#include <PID_v1.h>
#include <TimerOne.h>
#include <EEPROM.h>

//Inputs and outputs ports declarations
const int motor_pwm_pin = 4;
const int motor_dir1_pin = 26;
const int motor_dir2_pin = 27;
const int sensor1_trig_pin = 28;  //Unused sensor
const int sensor1_echo_pin = 29;
const int sensor2_trig_pin = 30;  //Unused sensor
const int sensor2_echo_pin = 31;
const int sensor3_trig_pin = 32;  //Unused sensor
const int sensor3_echo_pin = 33;
const int sensor4_trig_pin = 36;  //Front sensor
const int sensor4_echo_pin = 37;
const int sensor5_trig_pin = 38;  //Side sensor
const int sensor5_echo_pin = 39;
const int sensor6_trig_pin = 40;  //Back sensor
const int sensor6_echo_pin = 41;
const int servo_pin = 34;
const int inductive_sensor1_pin = 2;
const int inductive_sensor2_pin = 3;
const int button_pin = 42;

//Constants being used
const int MAX_DISTANCE = 150; //Max distance for the ultrasonic sensor
const double wheel_circunference = 0.4; //Radius of the wheel in meters
const double parking_space_needed = 0.5;  //parking space needed in meters, should be around 0.8

//Other global variables initialization
double desired_speed = 0.5; //Desired speed in m/s
int motor_speed = 170;
long distance1 = 0;
long distance2 = 0;
long distance3 = 0;
long distance4 = 0;
long distance5 = 0;
long distance6 = 0;
float parking_space_measured = 0.00f;
long parking_space = 0;
long minimum__distance = 10;
volatile long one_twentieth_revolutions1 = 0;
volatile long one_twentieth_revolutions2 = 0;
volatile int rotation_difference = 0;
float current_speed = 0.5f;
int servo_straight_angle = 90;
int eeAddress = 0;   //Location we want the data to be put.


// NewPing sonar1(sensor1_trig_pin, sensor1_echo_pin, MAX_DISTANCE);
// NewPing sonar2(sensor2_trig_pin, sensor2_echo_pin, MAX_DISTANCE);
// NewPing sonar3(sensor3_trig_pin, sensor3_echo_pin, MAX_DISTANCE);
NewPing sonar4(sensor4_trig_pin, sensor4_echo_pin, MAX_DISTANCE);
NewPing sonar5(sensor5_trig_pin, sensor5_echo_pin, MAX_DISTANCE);
NewPing sonar6(sensor6_trig_pin, sensor6_echo_pin, MAX_DISTANCE);
Servo servo;

//Get distances from all the ultrasonic sensors, sensors from 1 to 3 are not being used
void getAllDistances(){
  // distance1 = sonar1.ping_cm();
  // distance2 = sonar2.ping_cm();
  // distance3 = sonar3.ping_cm();
  distance4 = sonar4.ping_cm();
  distance5 = sonar5.ping_cm();
  distance6 = sonar6.ping_cm();
}


void moveForward(){
  digitalWrite(motor_dir1_pin, 1);
  digitalWrite(motor_dir2_pin, 0);
  analogWrite(motor_pwm_pin, motor_speed);
}

void moveBackwards(){
  digitalWrite(motor_dir1_pin, 0);
  digitalWrite(motor_dir2_pin, 1);
  analogWrite(motor_pwm_pin, motor_speed);
}

void brake(){
  digitalWrite(motor_dir1_pin, 1);
  digitalWrite(motor_dir2_pin, 1);
  analogWrite(motor_pwm_pin, 256);
}

void coast(){
  digitalWrite(motor_dir1_pin, 0);
  digitalWrite(motor_dir2_pin, 0);
  analogWrite(motor_pwm_pin, 0);
}

//From a pre defined position, park in a pre defined spot, without using any sensors
void preDefinedparalelPark(){
  servo.write(servo_straight_angle + 30);
  delay(1000);
  moveBackwards();
  delay(1200);
  brake();
  delay(1000);
  servo.write(servo_straight_angle);
  delay(1000);
  moveBackwards();
  delay(800);
  brake();
  delay(1000);
  servo.write(servo_straight_angle - 30);
  delay(1000);
  moveBackwards();
  delay(1000);
  brake();
  delay(1000);
  servo.write(servo_straight_angle + 30);
  delay(1000);
  moveForward();
  delay(850);
  brake();
  delay(1000);
  servo.write(servo_straight_angle);
}

//Parks the car in the detected parking spot using the sensors
void sensoredParalelPark(){
  // servo.write(servo_straight_angle + 30);
  servo.write(130);
  delay(500);
  getAllDistances();
  while(distance5 < 140 and distance5 != 0){
    moveBackwards();
    getAllDistances();
  }
  delay(600);
  while(distance5 < 140 and distance5 != 0){
    moveBackwards();
    getAllDistances();
  }
  brake();
  delay(1000);
  brake();
  delay(1000);
  servo.write(50);
  delay(400);
  while(distance6 > 15 or distance6 == 0){
    moveBackwards();
    getAllDistances();
  }
  brake();
  delay(1000);
  servo.write(130);
  delay(500);
  moveForward();
  delay(1350);
  brake();
  delay(400);
  servo.write(90);
  delay(400);
  while(distance4 > 30 or distance4 == 0){
    moveForward();
    getAllDistances();
  }
  brake();
  digitalWrite(13, HIGH);
}

//Interrupt to count the first wheel rotation
void inductive_sensor1_interrupt_callback(){
  one_twentieth_revolutions1++;
}

//Interrupt to count the second wheel rotation
void inductive_sensor2_interrupt_callback(){
  one_twentieth_revolutions2++;
}

//Goes forward looking for a parking spot and then run the parking algorithm
void findAndPark(){
  getAllDistances();
  while(distance5 <= 35 and distance5 != 0){
    moveForward();
    getAllDistances();
  }
  unsigned long initial_time = millis();
  one_twentieth_revolutions2 = 0;
  delay(200);
  while(distance5 > 35 or distance5 == 0){
    moveForward();
    getAllDistances();
  }
  float time_delta = (millis() - initial_time)/1000.00f;
  parking_space_measured = time_delta * current_speed;
  delay(300);
  brake();

  //Save the values measured to the EEPROM for later analysis
  EEPROM.put(eeAddress, time_delta);
  eeAddress += sizeof(float);
  EEPROM.put(eeAddress, parking_space_measured);
  eeAddress += sizeof(float);

  if (parking_space_measured >= parking_space_needed) {
    delay(3000);
    sensoredParalelPark();
  } else {
    // findAndPark();
  }
}


//Interrupt to calculate and adjust the motor speed, and to detect car steering
void timerIsr(){
  Timer1.detachInterrupt();  //stop the timer
  double rotation1 = (float(one_twentieth_revolutions1) / 20);  // divide by number of holes in Disc
  double rotation2 = (float(one_twentieth_revolutions2) / 20);  // divide by number of holes in Disc
  current_speed = ((rotation1+rotation2)/2) * wheel_circunference;
  rotation_difference = one_twentieth_revolutions2 - one_twentieth_revolutions1;
  Timer1.attachInterrupt( timerIsr );  //enable the timer
  if (current_speed < desired_speed){
    if (motor_speed < 125){
      motor_speed += 10;
    }
  } else {
    if (motor_speed > 10){
      motor_speed -= 10;
    }
  }
}

//Function to adjust the angle of the servo motor so the car go straight
void adjustServoAngle(){
  if (rotation_difference != 0){
    servo_straight_angle += rotation_difference;
  }
  if (servo_straight_angle < 60){
    servo_straight_angle = 60;
  }
  else if (servo_straight_angle > 120){
    servo_straight_angle = 120;
  }
  one_twentieth_revolutions1=0;  //  reset counter to zero
  one_twentieth_revolutions2=0;  //  reset counter to zero
}

//Makes the car move in a straight line
void goStraight(){
  adjustServoAngle();
  servo.write(servo_straight_angle);    
  moveForward();
}

//Makes the car go forward for n meters based on the last measured speed
void moveNMeters(double n){
  goStraight();
  float required_time = n/current_speed;
  delay(required_time * 1000);
  brake();
}


void setup() {
  //Setting up the inputs and outputs
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_dir1_pin, OUTPUT);
  pinMode(motor_dir2_pin, OUTPUT);
  pinMode(sensor1_trig_pin, OUTPUT);
  pinMode(sensor1_echo_pin, INPUT);
  pinMode(sensor2_trig_pin, OUTPUT);
  pinMode(sensor2_echo_pin, INPUT);
  pinMode(sensor3_trig_pin, OUTPUT);
  pinMode(sensor3_echo_pin, INPUT);
  pinMode(sensor4_trig_pin, OUTPUT);
  pinMode(sensor4_echo_pin, INPUT);
  pinMode(sensor5_trig_pin, OUTPUT);
  pinMode(sensor5_echo_pin, INPUT);
  pinMode(sensor6_trig_pin, OUTPUT);
  pinMode(sensor6_echo_pin, INPUT);

  servo.attach(servo_pin);
  // servo.detach();

  Serial.begin(9600);

  // Speed measurement
  Timer1.initialize(500000); // set timer for 0.5sec
  // Timer1.attachInterrupt( timerIsr ); // enable the timer

  //Inductive sensor interruption initialization for measuring wheel rotations
  pinMode(inductive_sensor1_pin, INPUT);
  pinMode(inductive_sensor2_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(inductive_sensor1_pin), inductive_sensor1_interrupt_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(inductive_sensor2_pin), inductive_sensor2_interrupt_callback, RISING);
  one_twentieth_revolutions1 = 0;
  one_twentieth_revolutions2 = 0;

  //Start the servo in the correct angle
  servo.write(servo_straight_angle);

  digitalWrite(13, LOW);
}

void loop() {
  servo.write(servo_straight_angle);
  //When button is pressed, run the parking code
  if (digitalRead(button_pin)) {
    delay(3000);
    findAndPark();
    // sensoredParalelPark();
  }
}