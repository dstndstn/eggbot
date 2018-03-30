#include <math.h>

#include "Arduino.h"


#if defined(STANDALONE)
#include "Arduino.h"
#include "avr/io.h"

// From https://github.com/arduino/Arduino/blob/2bfe164b9a5835e8cb6e194b928538a9093be333/hardware/arduino/avr/cores/arduino/main.cpp
// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

int main() {
  init();
  initVariant();
  setup();
    for (;;)
        loop();
}
#endif

/* Hardware-wise, there are two stepper motors:
 - A spins the egg, and 
 - B rotates the pen

 A is a 4-winding model
 B is a bipolar model

 both scavenged from old floppy drives or other such old equipment
 during my youth.

 For microstepping, I use PWM on motor B, which is driven by an
 SN754410 H-bridge.

 Unfortunately, I misread the manual for that chip and thought I could
 PWM the Motor Enable pins, but that doesn't work well.  Instead, need
 to PWM the Forward and Reverse pins.  Since I only have 2 PWM channels
 available, I use two 74LS00 chips to AND the PWM Enable and
 Forward/Reverse bits, and on the SN754410 tie the Enable pins high.

 Pins:

 Motor A:
 * 2 -> A1
 * 4 -> A2
 * 5 -> A3
 * 6 -> A4

 Motor B:
 * 11 -> M1 enable
 *  3 -> M2 enable
 *
 * 7 -> M2 reverse
 * 8 -> M1 forward
 * 9 -> M2 forward
 * 10-> M1 reverse
 */

#define MOTOR_A_1 2
#define MOTOR_A_2 4
#define MOTOR_A_3 5
#define MOTOR_A_4 6

#define MOTOR_B2_R 7
#define MOTOR_B1_F 8
#define MOTOR_B2_F 9
#define MOTOR_B1_R 10

#define MOTOR_B1_EN 11
#define MOTOR_B2_EN  3


/**
 * PWM:
 *
 * In Arduino, timer0 used for delay()
 * timer1 used for servo
 * timer2 used for tone
 * 
 * Pins:
 * OC0A = PD6  --> Arduino pin  6
 * OC0B = PD5  --> Arduino pin  5
 * 
 * OC1A = PB1  --> Arduino pin  9
 * OC1B = PB2  --> Arduino pin 10
 * 
 * OC2A = PB3  --> Arduino pin 11 --> EN1
 * OC2B = PD3  --> Arduino pin  3 --> EN2
 * 
 *  Arduino Pins:
 *  PORTB: pins 8 to 13
 *  PORTC: analog input pins
 *  PORTD: pins 0 to 7
 * 
 */

#define STEPB1 7
#define STEPB2 8
#define STEPB3 9
#define STEPB4 10

#define PWMB1 3
#define PWMB2 11

#define PEN_UP   13
#define PEN_DOWN 12

void setup() {
    pinMode(PEN_UP,   OUTPUT);
    pinMode(PEN_DOWN, OUTPUT);

    digitalWrite(PEN_UP,   LOW);
    digitalWrite(PEN_DOWN, LOW);

    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_A_3, OUTPUT);
    pinMode(MOTOR_A_4, OUTPUT);

    digitalWrite(MOTOR_A_1, LOW);
    digitalWrite(MOTOR_A_2, LOW);
    digitalWrite(MOTOR_A_3, LOW);
    digitalWrite(MOTOR_A_4, LOW);

    pinMode(MOTOR_B1_EN, OUTPUT);
    pinMode(MOTOR_B1_F,  OUTPUT);
    pinMode(MOTOR_B1_R,  OUTPUT);
    pinMode(MOTOR_B2_EN, OUTPUT);
    pinMode(MOTOR_B2_F,  OUTPUT);
    pinMode(MOTOR_B2_R,  OUTPUT);

    digitalWrite(MOTOR_B1_F, LOW);
    digitalWrite(MOTOR_B1_R, LOW);
    digitalWrite(MOTOR_B2_F, LOW);
    digitalWrite(MOTOR_B2_R, LOW);

    // Timer 2: WMG2 = 0x3 (fast PWM), COM2[AB] = 0x2 (non-inverting PWM)
    //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

    // Timer 2: WMG2 = 0x1 (phase correct PWM), COM2[AB] = 0x2 (non-inverting PWM)
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);

    // Timer 2: prescaler = 1 (CS2 = 0x1)
    TCCR2B = _BV(CS20);

    // For a while I thought the PWM was too fast for the motor driver.
    // Slow values can result in audible drive signals to the motor!
    // Timer 2: prescaler = 8 (CS22=0, CS21=1, CS20=0)
    // TCCR2B = _BV(CS21);
    // Timer 2: prescaler = 32
    // TCCR2B = _BV(CS21) | _BV(CS20);
    // Timer 2: prescaler = 256
    // TCCR2B = _BV(CS22) | _BV(CS21);
    // The very slowest rate causes the motor to vibrate rather than move!
    // Timer 2: prescaler = 1024
    // TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
    // vibrates!

    OCR2A = 0;
    OCR2B = 0;

    Serial.begin(9600);
}

/*
float remap_theta(float t0) {
    float t = fmod(t0, M_PI/2.);
    if (t < 0.)
        t += M_PI/2.;
    const float knee = 0.471;
    const float off  = 0.653;
    const float a = 0.883;
    float dy = t - off;
    return t0 - t + knee + (dy >= 0 ? 1 : -1) * pow(abs(dy)/a, 2.5);
}
 */

int isign(int i) {
    if (i > 0)
        return 1;
    if (i < 0)
        return -1;
    return 0;
}

class Motor {
  public:
    Motor() :
        x(0),
        scale(1.0),
        offset(0.0)
        {}
     
    virtual void zero() {
      offset = x;
      x = 0;
    }

    void step(float dx) {
        x += dx;
        go_to(x);
    }

    virtual void go_to(float thex) = 0;

    float x;
    float scale;
    float offset;
};


class MotorB : public Motor {
public:
    MotorB() :
      Motor(),
      lastic(0),
      lastis(0)
      //step(0.01),
      //delay(1),
    {
    }

    float waveform(float phase) {
      // cos-like.
      phase = fmod(phase, 2.*M_PI);
      if (phase < 0)
        phase += 2.*M_PI;
      // remap to 0 to 4.
      phase *= 4. / (2.*M_PI);

      // triangle wave
      if (phase < 2)
        return 1 - phase;
      return (phase-2) - 1;

      /*
      float xx[] = { 0., 0.4, 0.8,  1.2,  1.6,  2.0 };
      float yy[] = { 1., 0.6, 0.4, -0.4, -0.6, -1.0 };

      for (int i=0; i<(sizeof(xx)/sizeof(float))-1; i++) {
        if ((phase >= xx[i]) && (phase <= xx[i+1])) {
          float dd = (yy[i+1] - yy[i]) / (xx[i+1] - xx[i]);
          return yy[i] + (phase - xx[i]) * dd;
        }
      }

      // triangle wave
      if (phase < 2) {
        if (phase < 0.5) {
          return (1 - phase*1.5);
        }
        if (phase < 1.5) {
          return (1 - (0.5*1.5) - (phase - 0.5)*0.5);
        }
        return (-0.25 - (phase - 1.5)*1.5);
        //return 1 - phase;
      }
      return (phase-2) - 1;
      */
    }
    
    virtual void go_to(float thex) {
        x = thex;
        float theta = (x - offset) * scale;
        //float s = sin(theta);
        //float c = cos(theta);

        float c = waveform(theta);
        float s = waveform(theta + 1.5*M_PI);

        //s = (s > 0 ? s*s : -s*s);
        //c = (c > 0 ? c*c : -c*c);
        //s = (s > 0 ? sqrt(s) : -sqrt(-s));
        //c = (c > 0 ? sqrt(c) : -sqrt(-c));

        int ic = (int)(255 * c);
        int is = (int)(255 * s);
        int sgn;
        sgn = isign(ic);
        // If we're flipping signs (powering up the other winding),
        // turn both directions off first.
        if (isign(lastic) != sgn) {
            digitalWrite(MOTOR_B1_R, LOW);
            digitalWrite(MOTOR_B1_F, LOW);
        }
        sgn = isign(is);
        if (isign(lastis) != sgn) {
            digitalWrite(MOTOR_B2_R, LOW);
            digitalWrite(MOTOR_B2_F, LOW);
        }

        // Now set PWM
        analogWrite(MOTOR_B2_EN, abs(is));
        analogWrite(MOTOR_B1_EN, abs(ic));

        // windings direction enable bits
        sgn = isign(ic);
        if (isign(lastic) != sgn) {
            if (sgn == 0) {
                digitalWrite(MOTOR_B1_R, LOW);
                digitalWrite(MOTOR_B1_F, LOW);
            } else if (sgn > 0) {
                digitalWrite(MOTOR_B1_R, LOW);
                digitalWrite(MOTOR_B1_F, HIGH);
            } else {
                digitalWrite(MOTOR_B1_F, LOW);
                digitalWrite(MOTOR_B1_R, HIGH);
            }
        }
        lastic = ic;

        sgn = isign(is);
        if (isign(lastis) != sgn) {
            if (sgn == 0) {
                digitalWrite(MOTOR_B2_R, LOW);
                digitalWrite(MOTOR_B2_F, LOW);
            } else if (sgn > 0) {
                digitalWrite(MOTOR_B2_R, LOW);
                digitalWrite(MOTOR_B2_F, HIGH);
            } else {
                digitalWrite(MOTOR_B2_F, LOW);
                digitalWrite(MOTOR_B2_R, HIGH);
            }
        }
        lastis = is;
        
    }

protected:
    int lastic;
    int lastis;
};

class MotorA : public Motor {
public:
    MotorA() :
      Motor(),
      last_theta(0),
      theta_offset(0),
      stepdelay(1)
      {}

    virtual void zero() {
      x = 0;
      offset = 0;
      theta_offset = last_theta;
    }

    virtual void go_to(float thex) {
        x = thex;
        float theta = (x - offset) * scale;
        int itheta = int(theta) + theta_offset;
        int pause = 0;
        // Step there -- at most one of these loops will run
        for (; last_theta < itheta; last_theta++) {
          if (pause)
            delay(stepdelay);
          motor_A_to(last_theta);
          pause = 1;
        }
        for (; last_theta > itheta; last_theta--) {
          if (pause)
            delay(stepdelay);
          motor_A_to(last_theta);
          pause = 1;
        }
    }
    float stepdelay;

protected:
    int last_theta;
    int theta_offset;

};



MotorB mb;
MotorA ma;


class Pen {
  public:
  Pen() :
  x(0),
  y(0),
  step(0.2),
  pause(2)
  {}

  void zero() {
    ma.zero();
    mb.zero();
    x = y = 0;
  }

  void go_to(float new_x, float new_y) {
    float dx = (new_x - x);
    float dy = (new_y - y);
    float dist = sqrt(dx*dx + dy*dy);
    int nsteps = ceil(dist / step);
    float sx = step * dx / dist;
    float sy = step * dy / dist;
    // all but last step -- full step size.
    for (int i=0; i<nsteps-1; i++) {
      ma.step(sy);
      mb.step(sx);
      x += sx;
      y += sy;
      delay(pause);
    }
    // last step -- go there!
    ma.step(new_x - x);
    mb.step(new_y - y);
  }

  void pen_up() {
      digitalWrite(PEN_UP, HIGH);
      delay(100);
  }

  void pen_down() {
      digitalWrite(PEN_UP, LOW);
      delay(100);
  }
  
  float x;
  float y;
  float step;
  float pause;
  
};

Pen pen;

int d = 1;
float theta = 0;
float DT = 2.*M_PI / 256.0;
float dtheta = DT;

int inited = 0;

float r = 0;
float dr = 2.*M_PI / 80;

int aa = 0;

void loop() {

  if (!inited) {
      digitalWrite(PEN_UP, HIGH);
      delay(100);

      float ds = 0.04;
      for (int i=0; i<1250; i++) {
        mb.step(-ds);
        delay(d);
      }
      /*
      for (int i=0; i<750; i++) {
        mb.step(ds);
        delay(d);
      }
      for (int i=0; i<500; i++) {
        mb.step(-ds);
        delay(d);
      }
      for (int i=0; i<1000; i++) {
        mb.step(ds);
        delay(d);
      }
      */
      for (int i=0; i<625; i++) {
        mb.step(ds);
        delay(d);
      }
      // about 20. in total sweep.
      
      mb.zero();
      mb.scale = -0.1;

      /*
      for (int i=0; i<200; i++) {
        mb.step(1.);
        delay(10);
      }
      for (int i=0; i<200; i++) {
        mb.step(-1.);
        delay(10);
      }
      */

      ma.stepdelay = 2;
      ma.go_to(400);
      ma.stepdelay = 1;
      ma.zero();
      ma.scale = 0.5;
      inited = 1;

      ma.scale *= 0.4;
      mb.scale *= 0.4;

      pen.go_to( 0, 0);
      pen.pen_down();

      Serial.print("eggbot> ");
      Serial.flush();
  }

  if (Serial.available() == 0) {
    delay(1);
    return;
  }

  char cmd[32];
  memset(cmd, 0, 32);
  int i;
  for (i=0; i<32-1; i++) {
    int r = Serial.read();
    if (r == -1)
      return;

    Serial.print("read: ");
    Serial.println((char)r);
    Serial.flush();

    if (r == ' ')
      break;
    if (r == '\n')
      break;
    cmd[i] = r;
  }
  float ydir = 0;
  float xdir = 0;
  Serial.print("cmd: '");
  Serial.print((const char*)cmd);
  Serial.println("'");
  if (strcmp(cmd, "up") == 0) {
    Serial.println("up ");
    ydir = 1;   
  } else if (strcmp(cmd, "down") == 0) {
    Serial.println("down ");
    ydir = -1;
  } else if (strcmp(cmd, "left") == 0) {
    Serial.println("left ");
    xdir = -1;
  } else if (strcmp(cmd, "right") == 0) {
    Serial.println("right ");
    xdir = 1;
  } else if (strcmp(cmd, "penup") == 0) {
    Serial.println("pen up ");
    pen.pen_up();
    return;
  } else if (strcmp(cmd, "pendown") == 0) {
    Serial.println("pen down ");
    pen.pen_down();
    return;
  } else {
    Serial.print("unknown command '");
    Serial.print(cmd);
    Serial.println("'");
    return;
  }
  float dist = 0;
  if ((xdir != 0) || (ydir != 0)) {
    dist = Serial.parseFloat();
  }
  if (dist != 0) {
    Serial.println(dist);
    pen.go_to(pen.x + xdir * dist, pen.y + ydir * dist);
  }
  while (1) {
     int r = Serial.read();
    Serial.print("read ");
    Serial.println(r);
    Serial.flush();
     if (r == '\n')
       return;
  }

  /*
  // V
  pen.go_to(50, -100);
  pen.go_to(100, 0);

  // I
  pen.pen_up();
  pen.go_to(125, 0);
  pen.pen_down();
  pen.go_to(125, -100);

  // V
  pen.pen_up();
  pen.go_to(150, 0);
  pen.pen_down();
  pen.go_to(200, -100);
  pen.go_to(250,  0);
  pen.pen_up();

  // I
  pen.go_to(275, 0);
  pen.pen_down();
  pen.go_to(275, -100);
  pen.pen_up();

  // J
  pen.go_to(50, 200);
  pen.pen_down();
  pen.go_to(50, 100);
  pen.go_to(0,  100);
  pen.go_to(0,  150);
  pen.pen_up();

  pen.go_to(0, 400);
  pen.zero();
  */
  
  /*
  pen.go_to(  0,   0);
  pen.go_to(100,   0);
  pen.go_to(100, 100);
  pen.go_to(  0, 100);
  pen.go_to(  0,   0);
  */

/*
  if (mb.x >= 2.*M_PI) {
      mb.go_to(2.*M_PI);
      delay(100);
      for (int i=0; i<800; i++) {
        ma.step(1.);
        delay(2 * d);
      }
      digitalWrite(PEN_UP, HIGH);    
      for (;;) {
         delay(1000);
      }
  }

      
    digitalWrite(PEN_UP, HIGH);
    delay(100);
    for (int i=0; i<5; i++) {
        ma.step(1);
        delay(10 * d);
    }
    digitalWrite(PEN_UP, LOW);
    delay(100);

    mb.step(dtheta);
    delay(10*d);

    ma.step(1);
    delay(20 * d);
    */
}


void motor_A_to(int i) {
  i = i % 8;
  if (i < 0)
    i += 8;
  switch (i) {
    case 0:
      digitalWrite(MOTOR_A_1, HIGH);
      digitalWrite(MOTOR_A_2,  LOW);
      digitalWrite(MOTOR_A_3,  LOW);
      digitalWrite(MOTOR_A_4,  LOW);
      break;
    case 1:
      digitalWrite(MOTOR_A_1, HIGH);
      digitalWrite(MOTOR_A_2, HIGH);
      digitalWrite(MOTOR_A_3,  LOW);
      digitalWrite(MOTOR_A_4,  LOW);
      break;
    case 2:
      digitalWrite(MOTOR_A_1,  LOW);
      digitalWrite(MOTOR_A_2, HIGH);
      digitalWrite(MOTOR_A_3,  LOW);
      digitalWrite(MOTOR_A_4,  LOW);
      break;
    case 3:
      digitalWrite(MOTOR_A_1,  LOW);
      digitalWrite(MOTOR_A_2, HIGH);
      digitalWrite(MOTOR_A_3, HIGH);
      digitalWrite(MOTOR_A_4,  LOW);
      break;
    case 4:
      digitalWrite(MOTOR_A_1,  LOW);
      digitalWrite(MOTOR_A_2,  LOW);
      digitalWrite(MOTOR_A_3, HIGH);
      digitalWrite(MOTOR_A_4,  LOW);
      break;
    case 5:
      digitalWrite(MOTOR_A_1,  LOW);
      digitalWrite(MOTOR_A_2,  LOW);
      digitalWrite(MOTOR_A_3, HIGH);
      digitalWrite(MOTOR_A_4, HIGH);
      break;
    case 6:
      digitalWrite(MOTOR_A_1,  LOW);
      digitalWrite(MOTOR_A_2,  LOW);
      digitalWrite(MOTOR_A_3,  LOW);
      digitalWrite(MOTOR_A_4, HIGH);
      break;
    case 7:
      digitalWrite(MOTOR_A_1, HIGH);
      digitalWrite(MOTOR_A_2,  LOW);
      digitalWrite(MOTOR_A_3,  LOW);
      digitalWrite(MOTOR_A_4, HIGH);
      break;
  }
}


