//#define LeftHall 2
//#define RightHall 3
#define pi 3.1416


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
const float HalfWidth = 300;

volatile unsigned long LeftHallCount = 0;
volatile unsigned long RightHallCount = 0;
volatile float x = 0;
volatile float y = 0;
volatile float bearing = 0;
volatile float new_bearing = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  //set up pins
//  pinMode (outputPin, OUTPUT);
//  pinMode (MotorSpeed, OUTPUT);
  pinMode (LeftHall, INPUT_PULLUP);
  pinMode (RightHall, INPUT_PULLUP); 
  pinMode (5, OUTPUT);

  // set up interrupts
  attachInterrupt(digitalPinToInterrupt(LeftHall), CountLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(RightHall), CountRight, RISING);
}

void loop() {
//  // put your main code here, to run repeatedly:
//  digitalWrite(5, HIGH);
////  Serial.println("HIGH");
//  delay(1000);
//  digitalWrite(5, LOW);
////  Serial.println("LOW");
//  delay(1000);
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
    Serial.println(LeftHallCount);
    new_bearing = bearing + (90*L)/(180*HalfWidth); // in radian
    if (new_bearing >= 6.283) {
      new_bearing = new_bearing - 6.283;
    }
    Serial.print("Bearing: ");
    Serial.println(new_bearing);
    x = x + HalfWidth*(cos(bearing) - cos(new_bearing));
    
    Serial.print("x coordinate: ");
    Serial.println(x);
    y = y + HalfWidth*(sin(new_bearing) - sin(bearing));
    
    Serial.print("y coordinate: ");
    Serial.println(y);
    bearing = new_bearing;
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
    Serial.println(RightHallCount);
    bearing = bearing - (90*L)/(pi*HalfWidth);
    
    x = x - HalfWidth*(1 - cos(bearing));
    
    y = y + HalfWidth*(tan(bearing) - sin(bearing));
}
