#define HALL_THRESHOLD 500
int pwm_pin = 6;
int hall_pin = A0;
const float cir = 518.36;

float checkFreq(int hallPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(pwm_pin, OUTPUT);
  pinMode(hall_pin, INPUT);
  Serial.begin(9600);
  analogWrite(pwm_pin, 200);
}

void mainloop(){
  
  Serial.println(checkFreq(hall_pin));
}


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


void loop() {
  mainloop();
}
