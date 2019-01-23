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

  if (new_bearing >= 6.283) {
      new_bearing = new_bearing - 6.283;
    }
  if (new_bearing <= -6.283)  {
    new_bearing = new_bearing + 6.283;
  }
}

void CountLeft() {
  LeftHallCount++;
//  if(foward() == true)  {
//    new_bearing = bearing + (90*L)/(180*HalfWidth);
//    y = y + HalfWidth*(sin(new_bearing) - sin(bearing));
//  }
//  else  {
//    new_bearing = bearing - (90*L)/(180*HalfWidth)
//    y = y - HalfWidth*(sin(new_bearing) - sin(bearing));
//  }
//  x = x + HalfWidth*(cos(bearing) - cos(new_bearing));

  Serial.print("Left Hall Count: ");
  Serial.println(LeftHallCount);

  // Change in bearing = (90*L*pi)/(pi*HalfWidth*180) = 0.115267
  new_bearing = bearing + 0.1153; // in radian
  
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
//    new_bearing = bearing - (90*L)/(180*HalfWidth);
//    y = y + HalfWidth*(sin(new_bearing) - sin(bearing));
//  }
//  else  {
//    new_bearing = bearing + (90*L)/(180*HalfWidth);
//    y = y - HalfWidth*(sin(new_bearing) - sin(bearing));
//  }
//  x = x - HalfWidth*(cos(bearing) - cos(new_bearing));

  Serial.print("Right Hall Count: ");
  Serial.println(RightHallCount);
  new_bearing = bearing - 0.1153; // in radian
  
  Serial.print("Bearing: ");
  Serial.println(new_bearing);
  
  x = x - HalfWidth*(cos(bearing) - cos(new_bearing));
  
  Serial.print("x coordinate: ");
  Serial.println(x);
  
  y = y + HalfWidth*(sin(new_bearing) - sin(bearing));
  
  Serial.print("y coordinate: ");
  Serial.println(y);
  bearing = new_bearing;


//  void updateOdom(int turn) {
//  if (turn>0){
//    leftHallCount++;
//    float prev_theta = theta;
//    
//    if (leftReverse){
//      theta = theta - THETA_DELTA;
//      y = y - ROBOT_HALF_WIDTH*(cos(theta) - cos(prev_theta));
//    x = x - ROBOT_HALF_WIDTH*(sin(prev_theta) - sin(theta));
//    }else{
//      theta = theta + THETA_DELTA;
//      y = y + ROBOT_HALF_WIDTH*(cos(theta) - cos(prev_theta));
//    x = x + ROBOT_HALF_WIDTH*(sin(prev_theta) - sin(theta));
//    }
//    
//  }else{
//    rightHallCount++;
//    float prev_theta = theta;
//    
//    if (rightReverse){
//      theta = theta + THETA_DELTA;
//      y = y - ROBOT_HALF_WIDTH*(cos(theta) - cos(prev_theta));
//    x = x + ROBOT_HALF_WIDTH*(sin(prev_theta) - sin(theta));
//    }else{
//      theta = theta - THETA_DELTA;
//      y = y + ROBOT_HALF_WIDTH*(cos(theta) - cos(prev_theta));
//    x = x - ROBOT_HALF_WIDTH*(sin(prev_theta) - sin(theta));
//    }
//    
//  }
//
//}
}
