#define HALL_THRESHOLD 100
#define PWM_LEFT 5
#define PWM_RIGHT 6
#define HALL_PIN_LEFT A0
#define HALL_PIN_RIGHT A1
#define CTRL_SHIFT_RATE 0.2
#define WHEEL_CIRCUMFERENCE 100
#define STARTING_DUTY 100
#define DUTY_MAX 150
#define DUTY_MIN 102


// Feedback Controller Globals
int duty_left;
int duty_right;

// Destination Marker Global
int pulse_count_left = 0;
int pulse_count_right = 0;

float freqDiff();
float checkFreq(int hallPin, bool left_wheel);
void speedControl(int mode);
void Forward(int destination);

void setup() {
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(HALL_PIN_LEFT, INPUT);
  pinMode(HALL_PIN_RIGHT, INPUT);
  Serial.begin(9600);
  analogWrite(PWM_LEFT, STARTING_DUTY);
}

// speedControl called in a loop until destination reached
void speedControl(int mode){
  /*
  duty_left: initial duty cycle for left wheel
  duty_right: starting duty cycle for right wheel
  mode: Forward(0), Turn Right(1), or Turn Left(-1)
  */
  // float left_freq = checkFreq(HALL_PIN_LEFT, true);
  // float right_freq = checkFreq(HALL_PIN_RIGHT, false);

  float freq_compare = freqDiff();

  if (freq_compare > 0){
    duty_left -= CTRL_SHIFT_RATE*freq_compare;
    duty_right += CTRL_SHIFT_RATE*freq_compare;
    if (duty_left > DUTY_MAX){
     duty_left = DUTY_MAX; 
    }else if (duty_left < DUTY_MIN){
      duty_left = DUTY_MIN;
    }
    if (duty_right > DUTY_MAX){
      duty_right = DUTY_MAX;
    }else if (duty_right < DUTY_MIN){
      duty_right = DUTY_MIN;
    }
    analogWrite(PWM_LEFT, duty_left);
    analogWrite(PWM_RIGHT, duty_right);
  }else if(freq_compare < 0){
    duty_left += CTRL_SHIFT_RATE*freq_compare;
    duty_right -= CTRL_SHIFT_RATE*freq_compare;
    analogWrite(PWM_LEFT, duty_left);
    analogWrite(PWM_RIGHT, duty_right);
  }
}

// Gives the freq differences between both wheels (+ve means left is faster)
float freqDiff(){
  // Left and right wheel variables
  bool left_wasLowLevel=false;
  bool left_secEdge = false;
  unsigned long left_first_edge_time = 0;
  unsigned long left_sec_edge_time = 0;
  int left_hall_val;
  bool left_freq_done = false;
  float left_freq;

  bool right_wasLowLevel=false;
  bool right_secEdge = false;
  unsigned long right_first_edge_time = 0;
  unsigned long right_sec_edge_time = 0;
  int right_hall_val;
  bool right_freq_done = false;
  float right_freq;


  while (!(left_freq_done && right_freq_done)){
    // Left wheel polling
    left_hall_val = analogRead(HALL_PIN_LEFT);
    //Serial.print("Left hall val: ");
    //Serial.println(left_hall_val);
    if (!left_freq_done){
      if (left_hall_val >= 100) {
        if (left_wasLowLevel == true) {
          left_wasLowLevel = false;  // Reset edge detector

          // Increment Pulse
          pulse_count_left++;

          // Time Rising Edge
          if (left_secEdge == false){
            left_first_edge_time = micros();
            left_secEdge = true;
          }else{
            left_sec_edge_time = micros();
            unsigned long duration = left_sec_edge_time - left_first_edge_time;
            left_freq = 1000000/duration;
            left_freq_done = true;
          }
        }
      }else {
        left_wasLowLevel = true;
      }
    }

    // Right Wheel Polling
    right_hall_val = analogRead(HALL_PIN_RIGHT);
    //Serial.print("Right hall val: ");
    //Serial.println(right_hall_val);
    if (!right_freq_done){
      if (right_hall_val >= 30) {
        if (right_wasLowLevel == true) {
          right_wasLowLevel = false;  // Reset edge detector

          // Increment Pulse
          pulse_count_right++;

          // Time Rising Edge
          if (right_secEdge == false){
            right_first_edge_time = micros();
            right_secEdge = true;
          }else{
            right_sec_edge_time = micros();
            unsigned long duration = right_sec_edge_time - right_first_edge_time;
            right_freq = 1000000/duration;
            right_freq_done = true;
          }
        }
      }else {
        right_wasLowLevel = true;
      }
    }
  }
  Serial.print("Left Freq: ");
  Serial.println(left_freq);
  Serial.print("Right Freq: ");
  Serial.println(right_freq);
  Serial.print("Diff: ");
  return (left_freq - right_freq);
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
    //Serial.println(hall_val);
    delayMicroseconds(1000);
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
//          return 10/00000/duration;
          Serial.print("Frequency: ");
          return duration;
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

  analogWrite(PWM_LEFT, STARTING_DUTY);
  analogWrite(PWM_RIGHT, STARTING_DUTY);
  duty_left = STARTING_DUTY;
  duty_right = STARTING_DUTY;
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
  //if (Serial.available() > 0) {
    // Forward driving Control Test
     //int destination = Serial.parseInt();
    // Forward(destination);
  //}
  // Check Frequency Code Test
  Serial.println(freqDiff());

    // Freqency Diff Test
    // Serial.print("Freq Diff: ");
    // Serial.println(freqDiff());
//  }/

}
