#define HALL_THRESHOLD 500
#define PWM_LEFT 5
#define PWM_RIGHT 6
#define HALL_PIN_LEFT A0
#define HALL_PIN_RIGHT A1
#define CTRL_SHIFT_RATE 5

int duty_left = 127;
int duty_right = 127;

float checkFreq(int hallPin);
void speedControl(int pin_left, int pin_right);

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
void speedControl(int pin_left, int pin_right){
  float left_freq = checkFreq(pin_left);
  float right_freq = checkFreq(pin_right);

  if (left_freq > right_freq){
    duty_left -= CTRL_SHIFT_RATE;
    duty_right += CTRL_SHIFT_RATE;
    analogWrite(pin_left, duty_left);
    analogWrite(pwm_right, duty_right);
  }else if(right_freq > left_freq){
    duty_left += CTRL_SHIFT_RATE;
    duty_right -= CTRL_SHIFT_RATE;
    analogWrite(pin_left, duty_left);
    analogWrite(pwm_right, duty_right);
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

void Forward(){
  while(dist_travelled < destination){
    speedControl(HALL_PIN_LEFT, HALL_PIN_RIGHT);
  }
}

void loop() {
  mainloop();
}
