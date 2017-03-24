/*
Developed By:
Mehedi Hasan Mukit    ,CSE-14, IUT
Hossain Mohammad Seym ,CSE-14, IUT

With any kind of distribution authors name must be included.
*/



/*
  Defines the threshold to differ black and white.
  As sensor value ranges from 0-1023, so the value 
  must be in between 0 to 1023.
*/
#define whiteThershold 100

#define numberOfSensor 5        // assuming we are using 5 sensors here.

int sensorValues[numberOfSensor]; // This array will hold the sensor readings
void setup() {
    //this must be include, else NO output will be seen on Serial Monitor
    Serial.begin(9600);
}


void loop() {
    // sensor function is called here. 
    // Everything must be modular for easy coding and debugging.
    sensor();
}

int sensor()
{
//taking readings from ananlog pins
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);
  sensorValues[4] = analogRead(A4);

  for (int i = 0; i < numberOfSensor; i++)
  {

    // prints raw value on Serial Monitor
    Serial.print(sensorValues[i]);
    Serial.print('\t');

    /* turning raw values(0 to 1023) into digital values(1/0)
     comparing with whiteThershold
     In my case, reading for white surface is always less
     than 100, so I defined whiteThershold as 100 and 
     coverted white as '1' and black color as '0'
     for future purpose.
    */
    if ( sensorValues[i] < whiteThershold )
    {
      sensorValues[i] = 1;
    }
    else sensorValues[i] = 0;

    // uncomment it to see digital values and comment the line
    // 43- 44 to stop printing raw values
    //        Serial.print(sensorValues[i]);
    //        Serial.print('\t');
  }
  // print new line on Serial Monitor.
  Serial.println();
}
