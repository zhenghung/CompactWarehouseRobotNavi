int analogPin = 3;
int outputPin = 11;
int val; 
int thresh = 500;
int count = 0;
boolean fromLow = true;
const float cir = 518.36; // circumference in mm
float dist = 2000;
float pulse_count;
int state = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(outputPin, OUTPUT);
  pulse_count = (dist/cir)*15;
  
  
}

void loop()
{
  if (state == 0){
    Serial.println("START");
    Serial.println(pulse_count);
    count = 0;
    state = 1;
  }

  else if (state == 1){
    val = analogRead(analogPin);
    //Serial.println(val);
    
    if (val >= thresh){
      digitalWrite(outputPin, HIGH);
      if (fromLow == true)
      {
        Serial.println("change to HIGH");
        count++;
        fromLow = false;
      }
    }
    else {
      digitalWrite(outputPin, LOW);
      fromLow = true;
    }
  
    if (count >= pulse_count){
      state = 2;
    }
  
    Serial.print("COUNT");
    Serial.println(count);
  }

  else if (state == 2){
    Serial.println("STOP");
  }
}
