

#define LeftHall 2
#define RightHall 3
#define pi 3.14159


int val;  // used to store hall sensor values
int thresh = 500; // threshold value set for hall sensor values
int count = 0;  // counter for hall sensor pulses detected
boolean fromLow = true; 
const float cir = 518.36; // circumference of wheel in mm
float dist; // distance to travel
float pulse_count;  // converting distance to travel into number of pulses where 518.36mm is equivalent to 15 pulses
int state = 0; 
float L = 69.16;  // distance travelled in one hall sensor reading period

const int LeftHall = 2;
const int RightHall = 3;

volatile unsigned long LeftHallCount = 0;
volatile unsigned long RightHallCount = 0;
volatile int x = 0;
volatile int y = 0;
volatile float bearing = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //set up pins
  pinMode (outputPin, OUTPUT);
  pinMode (MotorSpeed, OUTPUT);
  pinMode (LeftHall, INPUT_PULLUP);
  pinMode (RightHall, INPUT_PULLUP);

  // set up interrupts
  attachInterrupt(digitalPinToInterrupt(LeftHall), CountLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(RightHall), CountRight, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:

}

void CountLeft() {
  LeftHallCount++;
//  if(foward() == true)  {
//    bearing = bearing + (90*L)/(pi*HalfWidth);
//    x = x + HalfWidth*(1 - cos(bearing));
//    y = y + HalfWidth*(tan(bearing) - sin(bearing)); 
//  }
//  else  {
//    bearing = bearing - (90*L)/(pi*HalfWidth);
//    x = x + HalfWidth*(1 - cos(bearing));
//    y = y - HalfWidth*(tan(bearing) - sin(bearing));
//  }
    bearing = bearing + (90*L)/(pi*HalfWidth);
    x = x + HalfWidth*(1 - cos(bearing));
    y = y + HalfWidth*(tan(bearing) - sin(bearing));
}

void CountRight() {
  RightHallCount++;
//  if(foward() == true)  {
//    bearing = bearing - (90*L)/(pi*HalfWidth);
//    x = x - HalfWidth*(1 - cos(bearing));
//    y = y + HalfWidth*(tan(bearing) - sin(bearing)); 
//  }
//  else  {
//    bearing = bearing + (90*L)/(pi*HalfWidth);
//    x = x - HalfWidth*(1 - cos(bearing));
//    y = y - HalfWidth*(tan(bearing) - sin(bearing));
//  }
    bearing = bearing - (90*L)/(pi*HalfWidth);
    x = x - HalfWidth*(1 - cos(bearing));
    y = y + HalfWidth*(tan(bearing) - sin(bearing));
}
