#define FIRST_MOTOR_STEP_PIN 45
#define FIRST_MOTOR_DIR_PIN 33
#define FIRST_MOTOR_5V 47
#define FIRST_MOTOR_ENABLE_PIN 35

#define SECOND_MOTOR_STEP_PIN 49
#define SECOND_MOTOR_DIR_PIN 51
#define SECOND_MOTOR_ENABLE_PIN 53

#define THIRD_MOTOR_IN1_PIN 37
#define THIRD_MOTOR_IN2_PIN 39
#define THIRD_MOTOR_IN3_PIN 41
#define THIRD_MOTOR_IN4_PIN 43

#define CW false
#define CCW true



volatile long firstMotorSteps = 0; // change for doing steps. Negative number = CW. 800 = 1 rotation
void configureFirstMotor()
{
  pinMode(FIRST_MOTOR_STEP_PIN, 1);
  pinMode(FIRST_MOTOR_DIR_PIN, 1);
  pinMode(FIRST_MOTOR_5V, 1);
  pinMode(FIRST_MOTOR_ENABLE_PIN, 1);
  configureTimer4();
}

void configureTimer4()
{
  cli();
  TCCR4A = 0;
  TCCR4B = 0;
  TCCR4B |= 0b00000100;
  TIMSK4 |= 0b00000010;
  OCR4A = 45; // 1388 times per second, ~1.75 revolutions per second
  sei();
}

void doFirstMotorStep()
{
  digitalWrite(FIRST_MOTOR_ENABLE_PIN, 1);
  digitalWrite(FIRST_MOTOR_5V, 1);
  digitalWrite(FIRST_MOTOR_DIR_PIN, firstMotorSteps < 0);
  digitalWrite(FIRST_MOTOR_STEP_PIN, !digitalRead(FIRST_MOTOR_STEP_PIN));
  firstMotorSteps += (firstMotorSteps < 0) ? 1 : -1;
}

ISR(TIMER4_COMPA_vect)
{
  if(firstMotorSteps)
    doFirstMotorStep();
  TCNT4 = 0;
}



volatile long secondMotorSteps = 0; // change for doing steps. Negative number = CW. 1600 = 1 rotation
void configureSecondMotor()
{
  pinMode(SECOND_MOTOR_STEP_PIN, 1);
  pinMode(SECOND_MOTOR_DIR_PIN, 1);
  pinMode(SECOND_MOTOR_ENABLE_PIN, 1);
  configureTimer3();
}

void configureTimer3()
{
  cli();
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3B |= 0b00000100;
  TIMSK3 |= 0b00000010;
  OCR3A = 30; //2083 times per second, ~1.3 revolutions per second
  sei();
}

void doSecondMotorStep()
{
  digitalWrite(SECOND_MOTOR_ENABLE_PIN, 0);
  digitalWrite(SECOND_MOTOR_DIR_PIN, secondMotorSteps < 0);
  digitalWrite(SECOND_MOTOR_STEP_PIN, !digitalRead(SECOND_MOTOR_STEP_PIN));
  secondMotorSteps += (secondMotorSteps < 0) ? 1 : -1;
}

ISR(TIMER3_COMPA_vect)
{
  if(secondMotorSteps)
    doSecondMotorStep();
  TCNT3 = 0;
}



bool dir;
long steps;

double speed;
volatile unsigned long interval;
const int actions[4] = {0b1001, 0b0101, 0b0110, 0b1010};
int action = 0;

double maxSpeed = 250.0;
int acceleration = 0;
long stepsBeforeDeaccelerate = 0;
int coefficient = 1;

void doStep(const bool (&modes)[4])
{
  digitalWrite(THIRD_MOTOR_IN1_PIN, modes[0]); 
  digitalWrite(THIRD_MOTOR_IN2_PIN, modes[1]);
  digitalWrite(THIRD_MOTOR_IN3_PIN, modes[2]);
  digitalWrite(THIRD_MOTOR_IN4_PIN, modes[3]); 
}

bool step()
{
  int current = actions[dir ? action : action ^ 0b11];
  bool modes[4];
  for(int i = 3; i >= 0; --i, current >>= 1)
    modes[i] = current & 1;
  doStep(modes);
  action = (action + 1) % 4;
  --steps;
  if(acceleration)
  {
    if(coefficient)
    {
      speed += coefficient * acceleration / 1000000.0 * interval;
      interval = 1000000.0 / speed;
    }
    if(speed >= maxSpeed && coefficient)
    {
      coefficient = 0;
      stepsBeforeDeaccelerate = stepsBeforeDeaccelerate * 2 - steps + 2;
    }
    if(steps == stepsBeforeDeaccelerate)
      coefficient = -1;
    configureTimer();
  }
  return steps;
}

void setSteps(long msteps = 0, bool mdir = CW, double mspeed = 50.0, double mmaxSpeed = 200.0, int maccel = 50)
{
  steps = msteps;
  dir = mdir;
  speed = mspeed;
  if(speed <= 1.0)
    speed = 1.0;
  maxSpeed = mmaxSpeed;
  if(maxSpeed < speed)
    maxSpeed = speed;
  acceleration = maccel;

  interval = 1000000.0 / speed;
  coefficient = 1;
  stepsBeforeDeaccelerate = steps / 2;
  action = 0;

  configureTimer();
}

void configureTimer()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= 0b00000100;
  TIMSK1 |= 0b00000010;
  OCR1A = interval / 16;
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;
  if(steps)
    step();
}

void configureThirdMotor()
{
  pinMode(THIRD_MOTOR_IN1_PIN, 1);
  pinMode(THIRD_MOTOR_IN2_PIN, 1);
  pinMode(THIRD_MOTOR_IN3_PIN, 1);
  pinMode(THIRD_MOTOR_IN4_PIN, 1);
}

const int maxFirstLength = 4400;
const int maxSecondLength = 1600;
const int maxThirdLength = 300;

const double minFirstAngle = 55.0;
const double maxFirstAngle = 125.0;
const double minSecondAngle = 20.0;
const double maxSecondAngle = 60.0;
const double minThirdAngle = 0.0;
const double maxThirdAngle = 70.0;


const double firstAngleDiff = maxFirstAngle - minFirstAngle;
const double secondAngleDiff = maxSecondAngle - minSecondAngle;
const double thirdAngleDiff = maxThirdAngle - minThirdAngle;


double degreesToRadians(double angle)
{
  const double pi = 3.1415926535;
  return angle * pi / 180.0; 
}

double radiansToDegrees(const double& r)
{
  const double pi = 3.1415926535;
  return r * 180.0 / pi; 
}


double firstAngle = 125.0;
double secondAngle = 60.0;
double thirdAngle = 0.0;

long firstLength = 4400;
long secondLength = 1600;
long thirdLength = 0;

const double firstAnglePerStep = double(firstAngleDiff) / maxFirstLength;
const double secondAnglePerStep = double(secondAngleDiff) / maxSecondLength;
const double thirdAnglePerStep = double(thirdAngleDiff) / maxThirdLength;

void calculateCoords(double& x, double& y, double& z)
{
  x = calculateX(firstAngle, secondAngle, thirdAngle);
  y = calculateY(firstAngle, secondAngle, thirdAngle);
  z = calculateZ(firstAngle, secondAngle, thirdAngle);
}

long L1 = 370;
long L2 = 440;

double calculateX(const double& angle1, const double& angle2, const double& angle3)
{
  double R = L1*cos(degreesToRadians(angle2)) + L2*sin(degreesToRadians(angle1 - 90 + angle2));
  return R*cos(degreesToRadians(angle3));
}

double calculateY(const double& angle1, const double& angle2, const double& angle3) //sin(a3)*(L1*cos(a1) + L2*cos(a2 + a1 - 180));
{
  double h = L1*sin(degreesToRadians(angle2));
  return h - L2*cos(degreesToRadians(angle1 - 90 + angle2));
}

double calculateZ(const double& angle1, const double& angle2, const double& angle3) //L1*sin(a1) + L2*sin(a2 + a1 - 180)
{
  double R = L1*cos(degreesToRadians(angle2)) + L2*sin(degreesToRadians(angle1 - 90 + angle2));
  return R*sin(degreesToRadians(angle3));
}



void inverseKinematics(const double& x, const double& y, const double& z)
{
  long steps1;
  long steps2;
  long steps3;
  calculateInverseKinematics(steps1, steps2, steps3, x, y, z);
  doFirstMotorSteps(steps1);
  doSecondMotorSteps(steps2);
  doThirdMotorSteps(steps3);
}

void calculateInverseKinematics(long& steps1, long& steps2, long& steps3, const double& x, const double& y, const double& z)
{

  double angle1 = calculateFirstAngle(x, y, z);
  double angle2 = calculateSecondAngle(x, y, z);
  double angle3 = calculateThirdAngle(x, y, z);

  Serial.print("angle1: ");
  Serial.print(angle1);
  Serial.print("; angle2: ");
  Serial.print(angle2);
  Serial.print("; angle3: ");
  Serial.println(angle3);

  if(firstAngle > angle1)
    steps1 = (firstAngle - angle1) / firstAnglePerStep * -4;
  else
    steps1 = (angle1 - firstAngle) / firstAnglePerStep * 4;

  if(secondAngle > angle2)
    steps2 = (secondAngle - angle2) / secondAnglePerStep * -8;
  else
    steps2 = (angle2 - secondAngle) / secondAnglePerStep * 8;

  if(thirdAngle > angle3)
    steps3 = (thirdAngle - angle3) / thirdAnglePerStep * -1;
  else
    steps3 = (angle3 - thirdAngle) / thirdAnglePerStep * 1;
//
//  Serial.print("steps1: ");
//  Serial.print(steps1);
//  Serial.print("; steps2: ");
//  Serial.print(steps2);
//  Serial.print("; steps3: ");
//  Serial.println(steps3);
}

double calculateFirstAngle(const double& x, const double& y, const double& z)
{
  double L3 = sqrt(sq(x) + sq(y) + sq(z));
  double numerator = sq(L1) + sq(L2) - sq(L3);
  double denominator = 2*L1*L2;
  return radiansToDegrees(acos(numerator / denominator));
}

double calculateSecondAngle(const double& x, const double& y, const double& z)
{
  double L3 = sqrt(sq(x) + sq(y) + sq(z));
  double R = sqrt(sq(L3) - sq(y));
  double numerator = sq(L1) + sq(L3) - sq(L2);
  double denominator = 2*L1*L3;
  double alpha = radiansToDegrees(acos(numerator / denominator));
  double delta = radiansToDegrees(acos(R / L3));
  return y < 0 ? alpha - delta : alpha + delta;
}

double calculateThirdAngle(const double& x, const double& y, const double& z)
{
  double L3 = sqrt(sq(x) + sq(y) + sq(z));
  double R = sqrt(sq(L3) - sq(y));
  return radiansToDegrees(acos(x / R));
}


void doFirstMotorSteps(const long steps)
{
  if((firstLength >= maxFirstLength && steps > 0) || (firstLength <= 0 && steps < 0))
    return;
  
  firstMotorSteps = steps;
  firstLength += steps / 4;
  firstAngle += (steps / 4) * firstAnglePerStep;
}

void doSecondMotorSteps(const long steps)
{
  if((secondLength >= maxSecondLength && steps > 0) || (secondLength <= 0 && steps < 0))
    return;
  
  secondMotorSteps = steps;
  secondLength += steps / 8;
  secondAngle += (steps / 8) * secondAnglePerStep;
}

void doThirdMotorSteps(const long steps)
{
  if((thirdLength >= maxThirdLength && steps > 0) || (thirdLength <= 0 && steps < 0))
    return;
  
  setSteps(abs(steps), (steps < 0), 75.0, 200.0, 50);
  thirdLength += steps;
  thirdAngle = thirdAnglePerStep*thirdLength;
}

double x = 0.0;
double y = 0.0;
double z = 0.0;

void printCoords()
{
  Serial.print("x = ");
  Serial.print(x, 4);
  Serial.print("; y = ");
  Serial.print(y, 4);
  Serial.print("; z = ");
  Serial.println(z, 4);
}

void setup() {
  configureFirstMotor();
  configureSecondMotor();
  configureThirdMotor();
  Serial.begin(115200);
}

void goBack()
{
  doFirstMotorSteps(35200);
  doSecondMotorSteps(12800);
  doThirdMotorSteps(-300);
}

void loop() {
  if(Serial.available())
  {
    long a, b, c;
    char input = Serial.read();
    switch(input)
    {
      case '1': goBack(); break;
      case '2': inverseKinematics(487.6691, 97.48441, 409.203); break;
      case '3': inverseKinematics(623.3257, 358.7779, 0); break;
      case '0': calculateCoords(x, y, z); printCoords(); break; 
    }
  }
}
