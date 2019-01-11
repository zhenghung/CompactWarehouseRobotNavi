#define HALL_THRESHOLD 500
#define PWM_LEFT 5
#define PWM_RIGHT 6
#define HALL_PIN_LEFT A0
#define HALL_PIN_RIGHT A1
#define CTRL_SHIFT_RATE 5
#define WHEEL_CIRCUMFERENCE 100

int duty_left = 0;
int duty_right = 0;
int pulse_count_left = 0;
int pulse_count_right = 0;

float checkFreq(int hallPin, bool left_wheel);
void speedControl(int mode);
void Forward(int destination);

void setup() {
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(HALL_PIN_LEFT, INPUT);
  pinMode(HALL_PIN_RIGHT, INPUT);
  Serial.begin(9600);
  analogWrite(PWM_LEFT, duty_left);
  analogWrite(PWM_RIGHT, duty_right);
}

// speedControl called in a loop until destination reached
void speedControl(int mode){
  /*
  duty_left: initial duty cycle for left wheel
  duty_right: starting duty cycle for right wheel
  mode: Forward(0), Turn Right(1), or Turn Left(-1)
  */
  float left_freq = checkFreq(HALL_PIN_LEFT, true);
  float right_freq = checkFreq(HALL_PIN_RIGHT, false);

  if (left_freq > right_freq){
    duty_left -= CTRL_SHIFT_RATE;
    duty_right += CTRL_SHIFT_RATE;
    analogWrite(HALL_PIN_LEFT, duty_left);
    analogWrite(HALL_PIN_RIGHT, duty_right);
  }else if(right_freq > left_freq){
    duty_left += CTRL_SHIFT_RATE;
    duty_right -= CTRL_SHIFT_RATE;
    analogWrite(HALL_PIN_LEFT, duty_left);
    analogWrite(HALL_PIN_RIGHT, duty_right);
  }
}



// checkFreq called to return current freq of rotation (delays one pulse)
float checkFreq(int hallPin, bool left_wheel){
  bool wasLowLevel=false;
  bool secEdge = false;
  unsigned long first_edge_time = 0;
  unsigned long sec_edge_time = 0;
  int hall_val;

  while (true){
    hall_val = analogRead(hallPin);
    if (hall_val >= HALL_THRESHOLD) {
      if (wasLowLevel == true) {
        wasLowLevel = false;  // Reset edge detector

        // Increment Pulses
        if (left_wheel){
          pulse_count_left++;
        }else{
          pulse_count_right++;
        }

        // Time Rising Edge
        if (secEdge == false){
          first_edge_time = micros();
          secEdge = true;
        }else{
          sec_edge_time = micros();
          unsigned long duration = sec_edge_time - first_edge_time;
          return 1000000/duration;
        }
      }
    }else {
      wasLowLevel = true;
    }
  }
}

void Forward(int destination){
  int pulse_count = (abs(destination)/WHEEL_CIRCUMFERENCE)*15;  // converting distance to number of pulse
  Serial.print("Pulse Count Required: ");
  Serial.println(pulse_count, DEC);

  while(pulse_count_left < pulse_count && pulse_count_right < pulse_count){
    speedControl(0);  //Forward Mode

    // Check Distance Travelled
    Serial.print("Pulse Count Left: ");
    Serial.println(pulse_count_left, DEC);
    Serial.print("Pulse Count Right: ");
    Serial.println(pulse_count_right, DEC);
  }
}


void Turn(int angle){
}


void loop() {
  if (Serial.available() > 0) {
    int destination = Serial.parseInt();
    Forward(destination);
  }
}
