/*
Developed By:
Mehedi Hasan Mukit    ,CSE-14, IUT
Hossain Mohammad Seym ,CSE-14, IUT
*/





//-------------------- Formatting Style Starts

/* defination of method: 
params:
return: 
*/

// demo formatting 
/* adds two integer 
params:
  a : first integer
  b : second integer
returns: 
  sum : sum of the values
*/
//---------------------- Formatting Style End





// Required Header files.
#include <EEPROM.h>
#include <QTRSensors.h>



//defining sensor threshold


#define whiteThreshold 250
#define Wpol 250
#define WpolFront 500


//defining printing format
#define one 0

//180  .85 4
//defining kp kd for PID
double kp = .85;
double kd = 4;


//defining minus and debug
#define minus 0
#define debug 0


//defining motor is enabled enable
char motorOn = 1;


// sort is defined is not done yet
char sort = 1;


//defining motor factor
#define factorL .9
#define factorR 1


//defining motor speed
#define baseSpeed 180
//another example 200 1 4


#define Z 0
#define leftSpeed baseSpeed + Z
#define rightSpeed baseSpeed + Z

#define LMS baseSpeed
#define RMS baseSpeed
#define Slow 0

int LS = leftSpeed, RS = rightSpeed;


// defining motor pins
#define inC 6
#define inD 7
#define enLeft 5

#define enRight 10
#define inA 8
#define inB 9


// defning sensor numbers
#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define numSensor NUM_SENSORS
#define noOfExtraSensor 5


// defining sensors 0 through 7 are connected to digital pins 35 through 49, respectively
QTRSensorsRC qtrrc((unsigned char[]) {
  35, 37, 39, 41, 43, 45, 47, 49
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
// defining sensors 0 through 7 are connected to digital pins 28 through 42, respectively
QTRSensorsRC front((unsigned char[]) {
  28, 30, 32, 34, 36, 38, 40, 42
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);


// defining setpoint
#define setpoint 350


int error, e, lastError = 0;


// a is an array for PID,extra is an array for DHUTTURI,f is an array for STOP
int a[numSensor], extra[noOfExtraSensor], f[numSensor];


//defining extrasensors name
int FL, FR;
int ML, MR;
int extremeL;

//defining flags
int leftFlag = 0;
int rightFlag = 0;


//defining array for maze solve
char finalArray[800];

//char finalArray[] = "SLLURLLULLRSRLSLLUSLLRSULLSRSULLULLLLSULULLRRLLLRLURLLR\0";
//char solved[] = "LSRRLLSRRSLLSRLLRRLRSRLSLRSSS\0";

char tempArray[800];
int finalIndex = 0, tempIndex = 0, pathIndex;


//defining allwhite
bool allwhite = false;


//defining millis values
unsigned int ti = millis();
unsigned int tf = millis();
unsigned int def = 0;
unsigned int dt = 0;


//defining setup
void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  pinMode(enLeft, OUTPUT);
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(enRight, OUTPUT);

  for (int i = 0; i < 20; i++)
  {
    motor(i * 5, i * 5);
    delay(5);
  }

  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}


//defining loop
void loop() {
  mapping();
  ultaghur();
  sorting();
  path();
  //  pidCalib();
  //  sensor();
}


//defining mapping
void mapping()
{

  while (true)
  {
    char turn = 1;
    //slowly adjusting the speed not ok


    //before going to follow line take ti ok
    leftFlag = 0;
    rightFlag = 0;


    // After Turn assuming last error is zero ok
    lastError = 0;
    ti = millis();
    followLine();
    //after returning from follow line take tf ok
    tf = millis();
    dt = tf - ti;
    if (dt > 1000) motorWithDelay(-50, -50, 50);
    else if (dt > 200) motorWithDelay(-50, -50, 10);
    else motorWithDelay(-10, -10, 5);
    if (0);
    //now the logic part
    //
    else if ((f[0] + f[1] + f[2] + f[3] + f[4] + f[5] + f[6] + f[7]) == 8 && (FL || FR))
    {
      motorWithDelay(60, 60, 20);
      finalArray[finalIndex] = '\0';
      return;
    }
    //logic for uturn ok
    else if (allwhite && (ML == 0 && MR == 0)) uturn();

    //logic for left turn normal ok
    else if ( ML || leftFlag || ((a[0] + a[1]) == 2)) {
      if (dt > 400) lTurn();
      else lTurnShort();
    }

    //logic for sturn ok
    else if (leftFlag == 0 && ((  a[4] + a[5] + a[6] + a[7]) > 2) && MR && ((f[0] + f[1] + f[2] + f[3] + f[4] + f[5] + f[6] + f[7]) > 0)) sTurn();

    //logic for rturn ok

    else if (
      (leftFlag == 0 && (MR || (  a[6] + a[7]) == 2)) &&
      ((f[0] + f[1] + f[2] + f[3] + f[4] + f[5] + f[6] + f[7]) == 0)
    )
    {
      rTurn();
    }

    else turn = 0;
    if (turn)
    {
      adjustSpeed();
    }
    //putting down the lights
    analogWrite(11, 0);
    analogWrite(12, 0);
    analogWrite(13, 0);
    //    motorWithDelay(0, 0, 200);
  }
}



//defining followline
void followLine()
{
  while (true)
  {
    //taking sensor value ok
    int pos = sensor();

    //flag for determining the turn ok
    if (FL || extremeL) leftFlag = 1;
    if (FR) rightFlag = 1;

    //breaking if found an intersection ok
    if (
      (ML && ((a[0] + a[1] + a[2]) == 3)  ) ||
      ( ML == 0 && (MR && ( (a[5] + a[6] + a[7]) == 3 ))) ||
      ( allwhite && (ML == 0 && MR == 0))
    )
    {
      Serial.println("breaking followline");
      //      motorWithDelay(0,0,2000);
      break;
    }


    //pid part ok
    error = (setpoint - pos);
    e = kp * error + kd * (error - lastError); //+ ki * integral;

    lastError = error;
    LS = leftSpeed - e;
    RS = rightSpeed + e;

    if (LS > LMS) LS = LMS;
    if (RS > RMS) RS = RMS;
    if (LS < minus) LS = minus;
    if (RS < minus) RS = minus;

    Serial.print(LS);
    Serial.print('\t');
    Serial.print(RS);
    Serial.print('\t');

    if (motorOn) motor(LS , RS);
    else motor(0, 0);
  }
}



//defining uturn
void uturn()
{
  analogWrite(13, 100);
  analogWrite(11, 100);

  finalArray[finalIndex] = 'U';
  finalIndex++;

  Serial1.print("U");

  //  motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[4] && MR == 0)break;
    motor(60, -65);
  }

  motorWithDelay(-200, 200, 20);
  //  motorWithDelay(0, 0, 2000);
}


//defining lturn
void lTurn()
{
  analogWrite(13, 100);

  finalArray[finalIndex] = 'L';
  finalIndex++;

  Serial1.print("L");

  //  motorWithDelay(0, 0, 2000);
  motor(-70, 80);
  delay(250);
  //  motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[0] == 0 && a[2])break;
    motor(-60, 55);
  }

  motorWithDelay(200, -200, 10);
  //        motorWithDelay(0, 0, 2000);
}


//defining lefttur short
void lTurnShort()
{
  analogWrite(13, 100);
  finalArray[finalIndex] = 'L';
  finalIndex++;

  Serial1.print("L");

  //md(0, 0, 2000);
  motor(-70, 90);
  delay(250);
  //  motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[0] == 0 && a[2])break;
    motor(-50, 55);
  }

  motorWithDelay(200, -200, 15);
  //    motorWithDelay(0, 0, 2000);
}


//defining sturn
void sTurn()
{
  analogWrite(13, 100);
  analogWrite(11, 100);
  analogWrite(12, 100);

  finalArray[finalIndex] = 'S';
  finalIndex++;

  Serial1.print("S");
  //  motorWithDelay(0, 0, 2000);

  motor(80, 80);
  delay(80);
  //  motorWithDelay(0, 0, 2000);
}


//defining rturn
void rTurn()
{
  analogWrite(11, 100);

  finalArray[finalIndex] = 'R';
  finalIndex++;
  Serial1.print("R");
  motorWithDelay(70, -45, 250);
  //    motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[5] && a[7] == 0)break;
    motor(70, -45);
  }

  motorWithDelay(-100, 100, 10);
  //  motorWithDelay(0, 0, 2000);
}











































//defining PID
void pid()
{
  int pos = sensor();

  error = (setpoint - pos);
  e = kp * error + kd * (error - lastError); //+ ki * integral;

  lastError = error;
  LS = leftSpeed - e;
  RS = rightSpeed + e;

  if (LS > LMS) LS = LMS;
  if (RS > RMS) RS = RMS;
  if (LS < minus) LS = minus;
  if (RS < minus) RS = minus;

  Serial.print(LS);
  Serial.print('\t');
  Serial.print(RS);
  Serial.print('\t');

  if (motorOn) motor(LS , RS);
  else motor(0, 0);
}


//defining sensor readings
int sensor()
{
  extra[0] = analogRead(A3);
  extra[1] = analogRead(A4);
  extra[2] = analogRead(A5);
  extra[3] = analogRead(A7);
  extra[4] = analogRead(A14);
  //extra sensor
  for (int i = 0; i < noOfExtraSensor; i++)
  {
    if (one == false) {
      Serial.print(extra[i]);
      Serial.print(' ');
    }

    if (extra[i] > whiteThreshold )
    {
      extra[i] = 1;
    }
    else {
      extra[i] = 0;
    }
    if (one == true) {
      Serial.print(extra[i]);
      Serial.print(' ');
    }
  }

  Serial.print('\t');

  ML = extra[0];
  FL = extra[1];
  FR = extra[2];
  MR = extra[3];
  extremeL = extra[4];
  //
  int pos, neu = 0, den = 0, blackCount = 0;
  //
  qtrrc.read(a);
  //PID sensor
  for (unsigned char i = 0; i < numSensor; i++)
  {
    if (one == false) {
      Serial.print(a[i]);
      Serial.print(' ');
    }
    if ( a[i] > Wpol )
    {
      a[i] = 1;
      neu = neu + i;
      den++;
      blackCount++;
    }
    else a[i] = 0;
    if (one == true) {
      Serial.print(a[i]);
      Serial.print(' ');
    }
  }

  Serial.print('\t');

  //
  front.read(f);
  //front STOP sensor
  for (unsigned char i = 0; i < numSensor; i++)
  {
    if (one == false) {
      Serial.print(f[i]);
      Serial.print(' ');
    }
    if ( f[i] > WpolFront )
    {
      f[i] = 1;
    }
    else f[i] = 0;
    if (one == true) {
      Serial.print(f[i]);
      Serial.print(' ');
    }
  }

  Serial.print("\t");

  //
  if (blackCount == 0) allwhite = true;
  else allwhite = false;

  if (den == 0) pos =  setpoint - lastError;
  else pos = (100 * neu) / den;

  Serial.print("\tpos");
  Serial.print(pos);
  Serial.println();
  return pos;
}


//defining both motor function
void motor(int LS, int RS)
{
  if (LS >= 0)motorL( 1, 0, LS * factorL);     //forward
  else motorL( 0, 1, (-1)*LS * factorL);     //backward

  if (RS >= 0)motorR( 1, 0, RS * factorR);     //forward
  else motorR( 0, 1, (-1)*RS * factorR);     //backward
}


//defining left motor function
void motorL(int ina, int inb, int mspeed)
{
  digitalWrite(inC, ina);
  digitalWrite(inD, inb);
  analogWrite(enLeft, mspeed);
}


//defining right motor function
void motorR(int ina, int inb, int mspeed)
{
  digitalWrite(inA, ina);
  digitalWrite(inB, inb);
  analogWrite(enRight, mspeed);
}


//definng both motor function with delay
void motorWithDelay(int L, int R, int del)
{
  motor(L, R);
  delay(del);
}


/*
Use to callibrate PID values with the help of Serial
params: -
returns: -
*/
void pidCalib()
{
  if (Serial1.available())
  {
    char c = Serial1.read();
    if (c == 'q')
    {
      kp += .05;
    }
    else if ( c == 'a')
    {
      kp -= .05;
    }
    else if ( c == 'w')
    {
      kd += .1;
    }
    else if (c == 's')
    {
      kd -= .1;
    }
    else if (c == 'n')
    {
      motorOn = 1;
    }
    else if (c == 'f')
    {
      motorOn = 0;
    }
  }
  Serial1.print(kp);
  Serial1.print(' ');
  Serial1.println(kd);
}






































void ultaghur()
{
  //md(0, 0, 1);
  motorWithDelay(60, -65, 900);

  for ( ; ; )
  {
    sensor();
    if (a[4] && MR == 0)break;
    motor(60, -70);
  }
  motorWithDelay(-200, 200, 15);
  motorWithDelay(0, 0, 1000);
}


//defining shortin for maze solve
void sorting()
{
  int flag = 1;
  while (flag > 0) {
    flag = 0;
    int i, j = 0;
    for (i = 0; i < strlen(finalArray); i++)
    {
      if (finalArray[i] == 'L' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'R')
      {
        tempArray[j] = 'U';
        j++;
        i = i + 2;
        flag++;
      }
      else if (finalArray[i] == 'L' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'S')
      {
        tempArray[j] = 'R';
        j++;
        i = i + 2;
        flag++;
      }
      else if (finalArray[i] == 'R' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'L')
      {
        tempArray[j] = 'U';
        j++;
        i = i + 2;
        flag++;
      }
      else if (finalArray[i] == 'S' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'L')
      {
        tempArray[j] = 'R';
        j++;
        i = i + 2;
        flag++;
      }
      else if (finalArray[i] == 'S' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'S')
      {
        tempArray[j] = 'U';
        j++;
        i = i + 2;
        flag++;
      }
      else if (finalArray[i] == 'L' && finalArray[i + 1] == 'U' && finalArray[i + 2] == 'L')
      {
        tempArray[j] = 'S';
        j++;
        i = i + 2;
        flag++;
      }
      else
      {
        tempArray[j] = finalArray[i];
        j++;
      }
    }

    tempArray[j] = '\0';
    if (flag > 0)
    {
      Serial.println(tempArray);
      strcpy(finalArray, tempArray);
      Serial.println(finalArray);
    }
  }
  int j = 0;
  for (int i = strlen(finalArray) - 1; i < strlen(finalArray); i--, j++)
  {
    if (finalArray[i] == 'L') tempArray[j] = 'R';
    else if (finalArray[i] == 'R') tempArray[j] = 'L';
    else tempArray[j] = finalArray[i];

  }
  tempArray[j] = '\0';
  strcpy(finalArray, tempArray);

}


void adjustSpeed()
{
  for (int i = 0; i < 30; i++) {
    motor(i * 5, i * 5);
    delay(2);
  }
}



// path will run untill end inside a loop
void path()
{

  while (true) {
    lastError = 0;
    //Adjusting the motor speed

    adjustSpeed();


    ti = millis();
    pathFollowLine();
    tf = millis();
    dt = tf - ti;
    if ((dt) > 300) motorWithDelay(-50, -50, 50);
    else motorWithDelay(-50, -50, 10);

    if (finalArray[pathIndex] == 'S')
    {
      sTurnPath();
      pathIndex++;
    }
    else if (finalArray[pathIndex] == 'R')
    {
      rTurnPath();
      pathIndex++;
    }

    else if (finalArray[pathIndex] == 'L')
    {
      lTurnPath();
      pathIndex++;
    }


    //stop At Last
    else if (finalArray[pathIndex] == '\0') {
      digitalWrite(13, 1);
      digitalWrite(12, 1);
      digitalWrite(11, 1);
      motorWithDelay(0, 0, 3000);
      digitalWrite(13, 0);
      digitalWrite(12, 0);
      digitalWrite(11, 0);
      motorWithDelay(0, 0, 10000);
    }
    digitalWrite(13, 0);
    digitalWrite(12, 0);
    digitalWrite(11, 0);
  }
}


//defining pathfollowline
void pathFollowLine()
{
  while (true)
  {
    //taking sensor value ok
    int pos = sensor();

    //flag for determining the turn ok
    if (FL) leftFlag = 1;
    if (FR) rightFlag = 1;

    //breaking if found an intersection ok
    if ((ML && a[0] && a[1] && a[2]) || (MR && a[5] && a[6] && a[7]))
    {
      Serial.println("breaking followline");
      //      motorWithDelay(0,0,2000);
      break;
    }


    //pid part ok
    error = (setpoint - pos);
    e = kp * error + kd * (error - lastError); //+ ki * integral;

    lastError = error;
    LS = leftSpeed - e;
    RS = rightSpeed + e;

    if (LS > LMS) LS = LMS;
    if (RS > RMS) RS = RMS;
    if (LS < minus) LS = minus;
    if (RS < minus) RS = minus;

    Serial.print(LS);
    Serial.print('\t');
    Serial.print(RS);
    Serial.print('\t');

    if (motorOn) motor(LS , RS);
    else motor(0, 0);
  }
}





//defining lturn
void lTurnPath()
{
  analogWrite(13, 100);

  Serial1.print("L");

  motor(-70, 80);
  delay(250);
  //  motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[0] == 0 && a[2])break;
    motor(-60, 55);
  }

  motorWithDelay(200, -200, 15);
  //    motorWithDelay(0, 0, 2000);
}


//defining lefttur short
void lTurnShortPath()
{
  analogWrite(13, 100);

  Serial1.print("L");

  motor(-70, 90);
  delay(250);
  //  motorWithDelay(0, 0, 2000);

  for ( ; ; )
  {
    sensor();
    if (a[0] == 0 && a[2])break;
    motor(-50, 50);
  }
  motorWithDelay(200, -200, 15);
  //    motorWithDelay(0, 0, 2000);
}


//defining sturn
void sTurnPath()
{
  analogWrite(13, 100);
  analogWrite(11, 100);
  analogWrite(12, 100);

  motor(80, 80);
  delay(80);
  //    motorWithDelay(0, 0, 2000);
}


//defining rturn
void rTurnPath()
{
  analogWrite(11, 100);
  motorWithDelay(60, -45, 300);

  for ( ; ; )
  {
    sensor();
    if (a[5] && a[7] == 0)break;
    motor(65, -45);
  }
  motorWithDelay(-100, 100, 10);
  //    motorWithDelay(0, 0, 2000);
}
