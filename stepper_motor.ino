#define IN1 37
#define IN2 39
#define IN3 41
#define IN4 43

#define CW true
#define CCW false

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
  digitalWrite(IN1, modes[0]); 
  digitalWrite(IN2, modes[1]);
  digitalWrite(IN3, modes[2]);
  digitalWrite(IN4, modes[3]); 
}

bool step()
{
  int current = actions[dir ? action : action ^ 0b11];
  bool modes[4];
  for(int i = 3; i >= 0; --i, current >>= 1)
    modes[i] = current & 1;
  doStep(modes);
  ++action;
  action %= 4;
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
  TCCR1B |= B00000100;
  TIMSK1 |= B00000010;
  OCR1A = interval / 16;
  sei();
}

ISR(TIMER1_COMPA_vect)
{
  TCNT1 = 0;
  if(steps)
    step();
}

void setup()
{
  Serial.begin(115200);
  pinMode(IN1, 1);
  pinMode(IN2, 1);
  pinMode(IN3, 1);
  pinMode(IN4, 1);
  setSteps(1000, CW, 20.0, 200.0, 20);
}

bool once = true;

void loop()
{
  if(!steps && once)
  {
    setSteps(1000, CCW, 20.0, 300.0, 20);
    once = false;
  }
}
