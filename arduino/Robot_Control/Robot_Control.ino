// constants that won't change used to set pin numbers
#define HALL_THRESHOLD 100

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
int thresh = 100; // threshold value set for hall sensor values
int count = 0;  // counter for hall sensor pulses detected
boolean fromLow = true; 
const float cir = 518.36; // circumference of wheel in mm
int dist; // distance to travel
float pulse_count;  // converting distance to travel into number of pulses where 518.36mm is equivalent to 15 pulses
int state = 0; 

void Forward();
void Backward();
void StopWhenReach();

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
      dist = Serial.parseInt();
      Serial.print("Received: ");
      Serial.println(dist, DEC);
		RobotState = 1;
      if (dist < 0){
        Backward();
      }else{
        Forward();
      }
      val = analogRead(LeftMotorHall);
      Serial.println(val);
      StopWhenReach();
    }

  }
}

// checkFreq called to return current freq of rotation (delays one pulse)
float checkFreq(int hallPin){
  bool wasLowLevel=false;
  bool secEdge = false;
  unsigned long first_edge_time = 0;
  unsigned long sec_edge_time = 0;
  int hall_val;

  while (true){
    hall_val = analogRead(hallPin);
    if (hall_val >= HALL_THRESHOLD) {
      if (wasLowLevel == true) {
        if (secEdge == false){
          first_edge_time = micros();
          secEdge = true;
        }else{
          sec_edge_time = micros();
          unsigned long duration = sec_edge_time - first_edge_time;
          return 1000000/duration;
        }
      }
      wasLowLevel = false;
    }else {
      wasLowLevel = true;
    }
  }
}


void Forward() {
  LeftWheelRelayState = 0;
  RightWheelRelayState = 0;
  digitalWrite(MotorSpeed, HIGH);
}

void Backward()  {
  LeftWheelRelayState = 1;
  RightWheelRelayState = 1;  
  digitalWrite(LeftWheelRelay, HIGH);
  digitalWrite(MotorSpeed, HIGH);
}

void StopWhenReach() {
  pulse_count = (abs(dist)/cir)*15;  // converting distance to number of pulse
  count = 0;
  while(count < pulse_count) {
    val = analogRead(A0);
    if (val >= thresh) {
      if (fromLow == true) {
        count++;
        Serial.println(count);
        fromLow = false;
     }
    }
    else {
      fromLow = true;
    }
  } 
  Serial.println("STOP");
  digitalWrite(MotorSpeed, LOW);
  RobotState = 0;
}

void ClockwiseRotation()  {
  LeftWheelRelayState = 0;
  RightWheelRelayState = 1;
  digitalWrite(MotorSpeed, HIGH);
}

void AntiClockwiseRotation()  {
  LeftWheelRelayState = 1;
  RightWheelRelayState = 0;
  digitalWrite(MotorSpeed, HIGH);
}
