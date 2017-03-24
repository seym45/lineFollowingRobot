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
#define integralLimit 100000
#define kp .4
#define kd 1
#define ki .0001
#define minus 0


//  This randomConstant defines base speed for both wheels
#define randomConstant 150

//base speeds for left and right wheel
#define leftBaseSpeed Z
#define rightBaseSpeed Z

//maximum allowed speeds 
#define leftMaxSpeed Z+30
#define rightMaxSpeed Z+30


//--------------------------------------------------- Advanced
// If you are beginner use factor  = 1

/* this factor is used to balance the motor speed.
  Let, to move the bot straight motor speed be like,
  leftMotorSpeed = 180 , rightMotorSpeed = 100.
  So, if we give a factor in motor functions 
*/
#define factor 1.1
// defining motor pins
#define inC 5
#define inD 6
#define enLeft 7

#define enRight 2
#define inA 4
#define inB 3

//Number of Sensor and setpoint
//#define NumSensor 7
#define setpoint 300

int LS = L, RS = R;
long long int integral = 0;
int a[9], f[7];
int fsum ;
void setup() {
  // speed of band
  Serial.begin(9600);
  pinMode(enLeft, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(enRight, OUTPUT);

  delay(500);
}

int i = 0;
void loop() {
  pid();
}

int error, e, lastError = 0, count = 0, flag = 0, c30 = 0;


void pid()
{
  int pos = sensor();
  error = (setpoint - pos);
  e = kp * error + kd * (error - lastError); //+ ki * integral;

  lastError = error;
  LS = leftBaseSpeed - e;
  RS = rightBaseSpeed + e;

  if (LS > LMS) LS = LMS;
  if (RS > RMS) RS = RMS;
  if (LS < minus) LS = minus;
  if (RS < minus) RS = minus;

  Serial.print(error);
  Serial.print('\t');
  Serial.print(LS);
  Serial.print('\t');
  Serial.print(RS);

  Serial.print('\t');


  motor(LS , RS * factor);
}


int sensor()
{
  a[0] = analogRead(A0);
  a[1] = analogRead(A1);
  a[2] = analogRead(A2);
  a[3] = analogRead(A3);
  a[4] = analogRead(A4);
  a[5] = analogRead(A5);


  int pos, neu = 0, den = 0;
  for (int i = 0; i < 7; i++)
  {
    Serial.print(f[i]);
    Serial.print('\t');
    if ( f[i] < W )
    {
      f[i] = 1;
      neu = neu + i;
      den++;
    }
    else f[i] = 0;

    fsum = fsum + f[i];

    //        Serial.print(f[i]);
    //        Serial.print('\t');
  }
  //  Serial.print('\t');


  //if (den == 0)pos = lastError + setpoint;
  if (den == 0)pos =  setpoint - lastError;
  else if (den == 7)pos = setpoint;
  else pos = (100 * neu) / den;
  Serial.print("\t sensor");
  Serial.print(pos);
  Serial.println();
  return pos;
}




void motor(int LS, int RS)
{
  if (LS >= 0)motorL( 1, 0, LS);       //forward
  else motorL( 0, 1, (-1)*LS);       //backward

  if (RS >= 0)motorR( 1, 0, RS);       //forward
  else motorR( 0, 1, (-1)*RS);       //backward
}
///left motor//
void motorL(int ina, int inb, int mspeed)
{
  digitalWrite(inC, ina);
  digitalWrite(inD, inb);
  analogWrite(enLeft, mspeed);
}
///right motor
void motorR(int ina, int inb, int mspeed)
{
  digitalWrite(inA, ina);
  digitalWrite(inB, inb);
  analogWrite(enRight, mspeed);
}



