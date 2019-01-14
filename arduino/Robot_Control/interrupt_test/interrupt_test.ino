#define HALL_THRESHOLD 500
#define PWM_LEFT 5
#define PWM_RIGHT 6
#define HALL_PIN_LEFT A0
#define HALL_PIN_RIGHT A1
#define CTRL_SHIFT_RATE 0.2
#define WHEEL_CIRCUMFERENCE 100
#define STARTING_DUTY 110
#define DUTY_MAX 150
#define DUTY_MIN 90


// Feedback Controller Globals
int duty_left;
int duty_right;

// Destination Marker Global
int pulse_count_left = 0;
int pulse_count_right = 0;

// Pulse Timer Global (Freq Measure)
unsigned long left_first_edge_time = 0;
unsigned long left_sec_edge_time = 0;
float left_freq;
unsigned long right_first_edge_time = 0;
unsigned long right_sec_edge_time = 0;
float right_freq;


void timeLeft();
void timeRight();
void speedControl(int mode);
void Forward(int destination);

void setup() {
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(HALL_PIN_LEFT, INPUT);
  pinMode(HALL_PIN_RIGHT, INPUT);
  Serial.begin(9600);

  // set up interrupts
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_LEFT), timeLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_RIGHT), timeRight, RISING);
}

void timeLeft(){
  pulse_count_left++;
  left_first_edge_time = left_sec_edge_time;
  left_sec_edge_time = micros();
  unsigned long duration = left_sec_edge_time - left_first_edge_time;
  left_freq = 1000000/duration;
}

void timeRight(){
  pulse_count_right++;
  right_first_edge_time = right_sec_edge_time;
  right_sec_edge_time = micros();
  unsigned long duration = right_sec_edge_time - right_first_edge_time;
  right_freq = 1000000/duration;
}

// speedControl called in a loop until destination reached
void speedControl(int mode){
  /*
  duty_left: initial duty cycle for left wheel
  duty_right: starting duty cycle for right wheel
  mode: Forward(0), Turn Right(1), or Turn Left(-1)
  */
  float freq_compare = left_freq - right_freq;

  // Modify duty cycle
  if (freq_compare > 0){
    duty_left -= CTRL_SHIFT_RATE*freq_compare;
    duty_right += CTRL_SHIFT_RATE*freq_compare;
  }else if(freq_compare < 0){
    duty_left += CTRL_SHIFT_RATE*freq_compare;
    duty_right -= CTRL_SHIFT_RATE*freq_compare;
  }

  // Cap out duty cycle
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

  // Write to pin
  analogWrite(PWM_LEFT, duty_left);
  analogWrite(PWM_RIGHT, duty_right);
}


void Forward(int destination){
  int pulses_required = (abs(destination)/WHEEL_CIRCUMFERENCE)*15;  // converting distance to number of pulse
  Serial.print("Pulse Count Required: ");
  Serial.println(pulses_required, DEC);

  analogWrite(PWM_LEFT, STARTING_DUTY);
  analogWrite(PWM_RIGHT, STARTING_DUTY);
  duty_left = STARTING_DUTY;
  duty_right = STARTING_DUTY;
  while(pulse_count_left < pulses_required && pulse_count_right < pulses_required){
    speedControl(0);  //Forward Mode

    // Check Distance Travelled
    Serial.print("Pulse Count Left: ");
    Serial.println(pulse_count_left, DEC);
    Serial.print("Pulse Count Right: ");
    Serial.println(pulse_count_right, DEC);
  }
  pulse_count_left=0;
  pulse_count_right=0;
}


void Turn(int angle){
}


void loop() {
  if (Serial.available() > 0) {
    int destination = Serial.parseInt();
    Forward(destination);
  }
}
