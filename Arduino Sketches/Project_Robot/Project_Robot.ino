// constants that won't change used to set pin numbers
#define LeftWheelRelay 2
#define RightWheelRelay 4
#define MotorSpeed 3
#define LeftMotorHall A0
#define RightMotorHall A1
#define outputPin 11

// variables that will change
int LeftWheelRelayState = 0;  // 0 = relay is state 0, wheels rotate in forward direction
int RightWheelRelayState = 0; // 1 = relay is state 1, wheels rotate in backward direction

int RobotState = 0; // Set the robot to only accept command when it is not moving
int RobotWidth = 0;
int val;  // used to store hall sensor values
int thresh = 500; // threshold value set for hall sensor values
int count = 0;  // counter for hall sensor pulses detected
boolean fromLow = true; 
const float cir = 518.36; // circumference of wheel in mm
float dist; // distance to travel
float pulse_count;  // converting distance to travel into number of pulses where 518.36mm is equivalent to 15 pulses
int state = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode (LeftMotorHall, INPUT);
  pinMode (RightMotorHall, INPUT);
  pinMode (outputPin, OUTPUT);
  pinMode (LeftWheelRelay, OUTPUT);
  pinMode (MotorSpeed, OUTPUT);
  pinMode (RightWheelRelay, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (RobotState == 0) {
    if (Serial.available() > 0) {
      dist = Serial.read();
      Serial.print("Received: ");
      Serial.println(dist, DEC);
      RobotState = 1;
      Forward();
      Stop();
    }
  }
}

void Forward() {
  LeftWheelRelayState = 0;
  RightWheelRelayState = 0;
  digitalWrite(MotorSpeed, 100);
}

void Backward()  {
  LeftWheelRelayState = 1;
  RightWheelRelayState = 1;  
  digitalWrite(MotorSpeed, 100);
}

void Stop() {
  pulse_count = (dist/cir)*15;  // converting distance to number of pulse
  
  if (state == 0){
    Serial.println("START");
    Serial.println(pulse_count);
    count = 0;
    state = 1;
  }
  else if (state == 1) {
    val = analogRead(LeftMotorHall);
    //Serial.println(val);
    if (val >= thresh) {
      digitalWrite(outputPin, HIGH);
      if (fromLow == true) {
        Serial.println("change to HIGH");
        count++;
        fromLow = false;
      }
    }
    else {
      digitalWrite(outputPin, LOW);
      fromLow = true;
    }
    if (count >= pulse_count) {
      state = 2;
    } 
    Serial.print("COUNT");
    Serial.println(count);
  }
  else if (state == 2) {
    Serial.println("STOP");
  }
  RobotState = 0;
}

void ClockwiseRotation()  {
  LeftWheelRelayState = 0;
  RightWheelRelayState = 1;
  digitalWrite(MotorSpeed, 100);
}

void AntiClockwiseRotation()  {
  LeftWheelRelayState = 1;
  RightWheelRelayState = 0;
  digitalWrite(MotorSpeed, 100);
}
