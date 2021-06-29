#include "LinkedList.h"
#include <PID_v1.h>

const byte ENCL = 2; // Left Encoder Interrupt Pin - INT 0
const byte ENCR = 3; // Right Encoder Interrupt Pin - INT 1

const byte ENL = 10; // Enable for Left Motor
const byte LM_P = 9;  // Left Motor +ve
const byte LM_N = 8;  // Left Motor -ve
const byte RM_N = 7;  // Right Motor -ve
const byte RM_P = 6;  // Right Motor +ve
const byte ENR = 5;  // Enable for Right Motor

const byte   IR_L = 11; // Left side IR sensor
const byte   IR_M = 12; // Middle IR sensor
const byte   IR_R = 13; // Right side IR sensor
int    cursor;
int    o_cur;
int    d_cur;
byte     state;
bool     solved;
byte     target_seq;
LinkedList<char> path;

// Integers for pulse counters
unsigned int counterL = 0;
unsigned int counterR = 0;

unsigned int timer1_counter = 0;

// Float for current RPM
double rpmL = 0.0;
double rpmR = 0.0;

// Float for the desired RPM
double rpm_setL = 0.0;
double rpm_setR = 0.0;

// Byte for PWM of each Motor
double pwmL = 0;
double pwmR = 0;

// PID controller for each Motor
double kp = 2, ki = 5, kd = 1;
PID pidL(&rpmL, &pwmL, &rpm_setL, kp, ki, kd, DIRECT);
PID pidR(&rpmR, &pwmR, &rpm_setR, kp, ki, kd, DIRECT);

// Float for number of slots in encoder disk
float diskslots = 20;

// Interrupt Service Routines

// Left Motor pulse count ISR
void ISR_countL()
{
  counterL++; // increment Left Motor counter value
}

// Right Motor pulse count ISR
void ISR_countR()
{
  counterR++; // increment Right Motor counter value
}

// Timer1 overflow
ISR(TIMER1_OVF_vect)
{
  TCNT1 = timer1_counter;

  float rpmL = (counterL / diskslots) * 600.00;
  counterL   = 0;

  float rpmR = (counterR / diskslots) * 600.00;
  counterR   = 0;
}

void setup()
{

  pinMode(ENL, OUTPUT); /* Right Motor Enable (ENA) */
  pinMode(LM_P, OUTPUT); /* Left Motor +ve terminal (IN4) */
  pinMode(LM_N, OUTPUT); /* Left Motor -ve terminal (IN3) */
  pinMode(RM_N, OUTPUT); /* Right Motor -ve terminal (IN2) */
  pinMode(RM_P, OUTPUT); /* Right Motor +ve terminal (IN1) */
  pinMode(ENR, OUTPUT); /* Left Motor Enable (ENB) */

  pinMode(IR_L, INPUT);
  pinMode(IR_M, INPUT);
  pinMode(IR_R, INPUT);

  // turn the PID on
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);

  /*----------- timer setup ---------------*/
  noInterrupts(); // disable all interrupts
  TCCR1A         = 0;
  TCCR1B         = 0;
  timer1_counter = 59286; // preload timer 65536-16MHz/256/2Hz (34286 for
        // 0.5sec) (59286 for 0.1sec)

  TCNT1 = timer1_counter; // preload timer
  TCCR1B |= (1 << CS12);  // 256 prescaler
  TIMSK1 |= (1 << TOIE1); // enable timer overflow interrupt
  interrupts();   // enable all interrupts
  /*----------- timer setup ---------------*/

  attachInterrupt(
      digitalPinToInterrupt(ENCL), ISR_countL,
      RISING); // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(
      digitalPinToInterrupt(ENCR), ISR_countR,
      RISING); // Increase counter 2 when speed sensor pin goes High

  solved     = false;
  target_seq = 0;
  cursor     = 0;
  path     = LinkedList<char>();
}

byte get_state()
{
  byte s = 0;
  s |= digitalRead(IR_L);
  s <<= 1;
  s |= digitalRead(IR_M);
  s <<= 1;
  s |= digitalRead(IR_R);
  return s;
}

void stop_motors()
{
  digitalWrite(LM_P, LOW);
  digitalWrite(LM_N, LOW);
  digitalWrite(RM_N, LOW);
  digitalWrite(RM_P, LOW);

  rpm_setL = 0;
  rpm_setR = 0;
}

void pid_motors()
{
  pidL.Compute();
  analogWrite(ENL, pwmL);

  pidR.Compute();
  analogWrite(ENR, pwmR);
}

void forward()
{
  digitalWrite(LM_P, HIGH);
  digitalWrite(LM_N, LOW);
  digitalWrite(RM_N, LOW);
  digitalWrite(RM_P, HIGH);

  rpm_setL = 60;
  rpm_setR = 60;
}

void step_forward()
{
  forward();

  int millis_old = millis();
  while (millis() - millis_old < 171)
    pid_motors();

  stop_motors();
}

void backward()
{
  digitalWrite(LM_P, LOW);
  digitalWrite(LM_N, HIGH);
  digitalWrite(RM_N, HIGH);
  digitalWrite(RM_P, LOW);

  rpm_setL = 60;
  rpm_setR = 60;
}

void step_backward()
{
  backward();

  int millis_old = millis();
  while (millis() - millis_old < 171)
    pid_motors();

  stop_motors();
}

void left()
{
  digitalWrite(LM_P, HIGH);
  digitalWrite(LM_N, LOW);
  digitalWrite(RM_N, LOW);
  digitalWrite(RM_P, HIGH);

  rpm_setL = 20;
  rpm_setR = 60;
}

void turnLeft()
{
  left();

  while (!(get_state() & 2))
    pid_motors();

  stop_motors();
}

void right()
{
  digitalWrite(LM_P, HIGH);
  digitalWrite(LM_N, LOW);
  digitalWrite(RM_N, LOW);
  digitalWrite(RM_P, HIGH);

  rpm_setL = 60;
  rpm_setR = 20;
}

void turnRight()
{
  right();

  while (!(get_state() & 2))
    pid_motors();

  stop_motors();
}

void rotate()
{
  digitalWrite(LM_P, HIGH);
  digitalWrite(LM_N, LOW);
  digitalWrite(RM_N, HIGH);
  digitalWrite(RM_P, LOW);

  rpm_setL = 60;
  rpm_setR = 60;
}

void rotate_back()
{
  rotate();

  while (!(get_state() & 2))
    pid_motors();

  stop_motors();
}

void targetReached()
{
  o_cur = 0; // define cursor = 0
  while (o_cur < path.size()) {
    if (path.get(o_cur) == 'D') {
      o_cur++; // skip 'D's
      continue;
    }

    // if non-'D' shift for each 'D' before
    for (d_cur = 0; d_cur < o_cur; ++d_cur) {
      if (path.get(d_cur) == 'D') {
        if (path.get(d_cur - 1) == 'L') {
          switch (path.get(o_cur)) {
          case 'L':
            path.set(o_cur, 'F');
            break;
          case 'F':
            path.set(o_cur, 'R');
            break;
          case 'R':
            path.set(o_cur, 'D');
            break;
          }
        } else if (path.get(d_cur - 1) == 'F') {
          switch (path.get(o_cur)) {
          case 'L':
            path.set(o_cur, 'R');
            break;
          case 'F':
            path.set(o_cur, 'D');
            break;
          }
        } else if (path.get(d_cur - 1) == 'R') {
          switch (path.get(o_cur)) {
          case 'L':
            path.set(o_cur, 'D');
            break;
          }
        }
      }
    }

    // if reached 'D', start deleting previous 'XD' pairs.
    if (path.get(o_cur) == 'D') {
      while (path.get(o_cur - 2) != 'D' &&
             path.get(o_cur - 1) == 'D') {
        path.remove(o_cur - 1);
        path.remove(o_cur - 2);
        o_cur -= 2;
      }
    }

    // forward cursor
    o_cur++;
  }

  // set it to be solved
  solved = true;
}

void loop()
{
  /* PID controller */
  pid_motors();

  if (solved) {
    state = get_state();
    if (state == 2) {
      // state == 010
      step_forward();
    } else if (state == 1) {
      // state == 001
      right();
    } else if (state == 4) {
      // state == 100
      left();
    } else {
      // decision need to be made
      switch (path.get(cursor)) {
      case 'L':
        turnLeft();
        break;
      case 'F':
        step_forward();
        break;
      case 'R':
        turnRight();
        break;
      }

      cursor++;
    }
  } else {
    // Maze not solved yet
    state = get_state();
    if (state == 7) {
      // state == 111
      target_seq++;
      if (target_seq < 3)
        step_forward();
      else
        targetReached();
    } else if (target_seq != 0) {
      // state != 111 means target sequence breaked
      while (target_seq--)
        step_backward();
      turnLeft();
      path.add('L');
    } else if (state == 6) {
      // state == 110
      turnLeft();
      path.add('L');
    } else if (state == 5) {
      // state == 101 is invalid
      rotate();
    } else if (state == 4) {
      // state == 100 robot needs to be calibrated to
      // the left
      left();
    } else if (state == 3) {
      // state == 011
      step_forward();

      if (get_state() & 2) {
        path.add('F');
      } else {
        step_backward();
        turnRight();
        path.add('R');
      }
    } else if (state == 2) {
      // state == 010
      forward();
    } else if (state == 1) {
      // state == 001 robot needs to be calibrated to
      // the right
      right();
    } else {
      // state == 000, Dead End
      rotate_back();
      path.add('D');
    }
  }
}
