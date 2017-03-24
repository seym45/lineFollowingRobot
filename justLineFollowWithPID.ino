/*
Developed By:
Mehedi Hasan Mukit    ,CSE-14, IUT
Hossain Mohammad Seym ,CSE-14, IUT

With any kind of distribution include the authors name.
*/










sudo#define W 100
#define integralLimit 100000


#define kp  .4//.3
#define kd 1
#define ki .000
#define minus 0

#define Z 150

//base speeds
#define L Z
#define R Z
//max speeds
#define factor 1.1
//150 -30

//160 -20
#define LMS Z+30
#define RMS Z+30


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
  LS = L - e;
  RS = R + e;

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

  f[0] = analogRead(A15);
  f[1] = analogRead(A14);
  f[2] = analogRead(A13);
  f[3] = analogRead(A12);
  f[4] = analogRead(A11);
  f[5] = analogRead(A10);
  f[6] = analogRead(A9);

  fsum = 0;
  //
  for (int i = 0; i < 6; i++)
  {
    //        Serial.print(a[i]);
    //        Serial.print('\t');

    if ( a[i] < W )a[i] = 1;
    else a[i] = 0;

    Serial.print(a[i]);
    Serial.print('\t');
  }
  //  Serial.print("fsum ");
  //  Serial.print(fsum);
  Serial.print('\t');
  Serial.print('\t');


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



