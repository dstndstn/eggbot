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

 Motor B:
 * 11 -> M1 enable
 * 2  -> M1 forward
 * 4  -> M1 reverse
 * 3  -> M2 enable
 * 5  -> M2 forward
 * 6  -> M2 reverse
 */

#define MOTOR_A_1 7
#define MOTOR_A_2 8
#define MOTOR_A_3 9
#define MOTOR_A_4 10

#define MOTOR_B1_EN 11
#define MOTOR_B1_F  2
#define MOTOR_B1_R  4
#define MOTOR_B2_EN 3
#define MOTOR_B2_F  5
#define MOTOR_B2_R  6

#define PEN_UP 12
#define PEN_DOWN 13

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

void setup() {
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
    digitalWrite(MOTOR_B1_EN, LOW);
    digitalWrite(MOTOR_B2_EN, LOW);

    pinMode(MOTOR_A_1, OUTPUT);
    pinMode(MOTOR_A_2, OUTPUT);
    pinMode(MOTOR_A_3, OUTPUT);
    pinMode(MOTOR_A_4, OUTPUT);

    digitalWrite(MOTOR_A_1, LOW);
    digitalWrite(MOTOR_A_2, LOW);
    digitalWrite(MOTOR_A_3, LOW);
    digitalWrite(MOTOR_A_4, LOW);

    pinMode(PEN_UP,  OUTPUT);
    digitalWrite(PEN_UP, LOW);
    pinMode(PEN_DOWN,  OUTPUT);
    digitalWrite(PEN_DOWN, LOW);

    if (1) {
    // Timer 2: WMG2 = 0x3 (fast PWM), COM2[AB] = 0x2 (non-inverting PWM)
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

    // Timer 2: WMG2 = 0x1 (phase correct PWM), COM2[AB] = 0x2 (non-inverting PWM)
    //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);

    // Timer 2: prescaler = 1 (CS2 = 0x1)
    TCCR2B = _BV(CS20);

    // For a while I thought the PWM was too fast for the motor driver.
    // Slow values can result in audible drive signals to the motor!
    // Timer 2: prescaler = 8 (CS22=0, CS21=1, CS20=0)
    //TCCR2B = _BV(CS21);
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
    }

    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
}

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

class MotorA : public Motor {
public:
    MotorA(int p1, int p2, int p3, int p4) :
      Motor(),
      last_theta(0),
      theta_offset(0),
      stepdelay(1),
      pin1(p1),
      pin2(p2),
      pin3(p3),
      pin4(p4)
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
          pins_to(last_theta);
          pause = 1;
        }
        for (; last_theta > itheta; last_theta--) {
          if (pause)
            delay(stepdelay);
          pins_to(last_theta);
          pause = 1;
        }
    }
    float stepdelay;

    void pins_to(int i) {
      i = i % 8;
        if (i < 0)
          i += 8;
      switch (i) {
      case 0:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2,  LOW);
        digitalWrite(pin3,  LOW);
        digitalWrite(pin4,  LOW);
        break;
      case 1:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3,  LOW);
        digitalWrite(pin4,  LOW);
        break;
      case 2:
        digitalWrite(pin1,  LOW);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3,  LOW);
        digitalWrite(pin4,  LOW);
        break;
      case 3:
        digitalWrite(pin1,  LOW);
        digitalWrite(pin2, HIGH);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4,  LOW);
        break;
      case 4:
        digitalWrite(pin1,  LOW);
        digitalWrite(pin2,  LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4,  LOW);
        break;
      case 5:
        digitalWrite(pin1,  LOW);
        digitalWrite(pin2,  LOW);
        digitalWrite(pin3, HIGH);
        digitalWrite(pin4, HIGH);
        break;
      case 6:
        digitalWrite(pin1,  LOW);
        digitalWrite(pin2,  LOW);
        digitalWrite(pin3,  LOW);
        digitalWrite(pin4, HIGH);
        break;
      case 7:
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2,  LOW);
        digitalWrite(pin3,  LOW);
        digitalWrite(pin4, HIGH);
        break;
      }
    }

protected:
    int last_theta;
    int theta_offset;

    int pin1;
    int pin2;
    int pin3;
    int pin4;

};


class MotorB : public Motor {
public:
    MotorB(int en1, int f1, int r1, int en2, int f2, int r2) :
      Motor(),
      lastic(0),
      lastis(0),
      enable1(en1),
      forward1(f1),
      reverse1(r1),
      enable2(en2),
      forward2(f2),
      reverse2(r2)
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
    }

    virtual void go_to(float thex) {
        x = thex;
        float theta = (x - offset) * scale;

        float c = waveform(theta);
        float s = waveform(theta + 1.5*M_PI);

        int ic = (int)(255 * c);
        int is = (int)(255 * s);
        int sgn;
        sgn = isign(ic);
        // If we're flipping signs (powering up the other winding),
        // turn both directions off first.
        if (isign(lastic) != sgn) {
            digitalWrite(forward1, LOW);
            digitalWrite(reverse1, LOW);
        }
        sgn = isign(is);
        if (isign(lastis) != sgn) {
            digitalWrite(forward2, LOW);
            digitalWrite(reverse2, LOW);
        }

        // Now set PWM
        analogWrite(enable1, abs(ic));
        analogWrite(enable2, abs(is));

        //digitalWrite(enable1, (ic > 0));
        //digitalWrite(enable2, (is > 0));

        // windings direction enable bits
        sgn = isign(ic);
        if (isign(lastic) != sgn) {
            if (sgn == 0) {
                digitalWrite(forward1, LOW);
                digitalWrite(reverse1, LOW);
            } else if (sgn > 0) {
                digitalWrite(reverse1, LOW);
                digitalWrite(forward1, HIGH);
            } else {
                digitalWrite(forward1, LOW);
                digitalWrite(reverse1, HIGH);
            }
        }
        lastic = ic;

        sgn = isign(is);
        if (isign(lastis) != sgn) {
            if (sgn == 0) {
                digitalWrite(forward2, LOW);
                digitalWrite(reverse2, LOW);
            } else if (sgn > 0) {
                digitalWrite(reverse2, LOW);
                digitalWrite(forward2, HIGH);
            } else {
                digitalWrite(forward2, LOW);
                digitalWrite(reverse2, HIGH);
            }
        }
        lastis = is;
        
    }

protected:
    int lastic;
    int lastis;

    int enable1;
    int forward1;
    int reverse1;
    int enable2;
    int forward2;
    int reverse2;
};

#if 0
class MotorB : public Motor {
public:
    MotorB(int en1, int f1, int r1, int en2, int f2, int r2) :
      Motor(),
      lastx(0),
      enable1(en1),
      forward1(f1),
      reverse1(r1),
      enable2(en2),
      forward2(f2),
      reverse2(r2)
    {
    }

    virtual void go_to(float thex) {
        x = thex;

        int ix = (int)x;
        if (ix == lastx)
          return;

        ix = ix % 4;
        if (ix < 0)
          ix += 4;
        lastx = ix;
        
        digitalWrite(enable1,  LOW);
        digitalWrite(forward1, LOW);
        digitalWrite(reverse1, LOW);
        digitalWrite(enable2,  LOW);
        digitalWrite(forward2, LOW);
        digitalWrite(reverse2, LOW);

        switch (ix) {
          case 0:
            digitalWrite(forward1, HIGH);
            digitalWrite(enable1,  HIGH);
            break;
          case 1:
            digitalWrite(forward2, HIGH);
            digitalWrite(enable2,  HIGH);
            break;
          case 2:
            digitalWrite(reverse1, HIGH);
            digitalWrite(enable1,  HIGH);
            break;
          case 3:
            digitalWrite(reverse2, HIGH);
            digitalWrite(enable2,  HIGH);
            break;
        }
    }

protected:
    int lastx;
    
    int enable1;
    int forward1;
    int reverse1;
    int enable2;
    int forward2;
    int reverse2;
};
#endif


MotorB mb(MOTOR_B1_EN, MOTOR_B1_F, MOTOR_B1_R,
          MOTOR_B2_EN, MOTOR_B2_F, MOTOR_B2_R);

MotorA ma(MOTOR_A_1, MOTOR_A_2, MOTOR_A_3, MOTOR_A_4);

int N = 0;

float da = 0.1;
float db = -0.12;

int d = 1;

// motor B: ~240 across the width of the egg
// motor A:  400 steps around the egg


// inkscape interface;
// 9600 baud
// v\r -> EBBx
// http://evil-mad.github.io/EggBot/ebb.html

void loop() {
  char linebuffer[32];
  if (Serial.available() > 0) {
    int nr = Serial.readBytesUntil('\r', linebuffer, sizeof(linebuffer)-1);
    linebuffer[nr] = '\0';
    if (nr == 0)
      return;
    String line(linebuffer);
    line.toUpperCase();
    if (line.startsWith("V") || line.startsWith("v")) {
      String reply = String("EBB LANG1 ") + line + String("\r\n");
      Serial.write(reply.c_str());
      //Serial.write("EBB LANG1 " + line + "\r\n");
    } else if (line.startsWith("QB")) {
      Serial.write("0\r\nOK\r\n");
    } else if (line.startsWith("SC,4,") || // pen up limit
               line.startsWith("SC,5,") || // pen down limit
               line.startsWith("SC,11,") || // pen up speed
               line.startsWith("SC,12,") || // pen down speed
               line.startsWith("EM,1,1")    // enable motor 1
               ) {
      // Pen up/down servo positions
      Serial.write("OK\r\n");
    } else if (line.startsWith("SP,")) {
       line = line.substring(3);
       if (line.startsWith("1")) {
          digitalWrite(PEN_UP, HIGH);
          delay(500);
       } else if (line.startsWith("0")) {
          //digitalWrite(PEN_UP, LOW);
          // in 10 steps of 50 ms each, bit-bang the PEN_UP motor from all-on to all-off.
          for (int i=0; i<10; i++) {
            for (int j=0; j<5; j++) {
              digitalWrite(PEN_UP, HIGH);
              delay(10-i);
              digitalWrite(PEN_UP, LOW);
              delay(i);
            }
          }
          delay(500);
          digitalWrite(PEN_DOWN, HIGH);
          delay(500);
          digitalWrite(PEN_DOWN, LOW);
       }
      Serial.write("OK\r\n");
    } else if (line.startsWith("SM,")) {
       line = line.substring(3);
       int comma = line.indexOf(',');
       String ss = line.substring(0, comma);
       long dur = ss.toInt();
       line = line.substring(comma+1);
       comma = line.indexOf(',');
       float axis1, axis2;
       if (comma == -1) {
         axis2 = 0.0;
         axis1 = line.toFloat();
       } else {
         ss = line.substring(0, comma);
         axis1 = ss.toFloat();
         line = line.substring(comma+1);
         axis2 = line.toFloat();
       }

      float scalea = 0.125;
      float scaleb = 0.208;
       
       axis1 /= dur;
       axis2 /= dur;
       while (dur > 0) {
         mb.step(axis1 * scaleb);
         ma.step(axis2 * scalea);
         dur--;
         delay(1);
       }
      Serial.write("OK\r\n");
       
    }
    // SM,3297,-122,1313
  }
}

void loopNothing() {
  int i;
  int j;
  int k;
  for (j=0; j<24; j++) {
    if (j % 2 == 1) {
      digitalWrite(PEN_UP, HIGH);
      delay(500);
      for (k=0; k<100; k++) {
        mb.step(-0.1);
        delay(d);
      }
      digitalWrite(PEN_UP, LOW);
      delay(500);
      continue;
    }

    for (k=0; k<10; k++) {
      /*
      for (i=0; i<4000; i++) {
        ma.step(da);
        delay(d);
      }
      */
      for (i=0; i<800; i++) {
        ma.step(0.5);

        mb.step(-1./800);

        delay(d);
      }
      //mb.step(-1.);
    }
  }
}

#if 0
int starting = 1;
void loop() {
  if (starting == 1) {
      digitalWrite(PEN_UP, HIGH);
      delay(100);
      starting = 2;
  }
  if (starting == 2) {
    mb.step(db);
    N += 1;
    if (N == 1000) {
      starting = 3;
      N = 0;

      digitalWrite(PEN_UP, LOW);
      delay(100);
    }
  }
  if (starting == 3) {
    ma.step(da);
    N += 1;
    if (N == 4000) {
      starting = 4;
      N = 0;
    }
  }
  delay(d);
}
#endif

/*
void loop() {

  if (N > 2000) {
      db *= -1;
      digitalWrite(PEN_UP, HIGH);
      delay(2000);
      digitalWrite(PEN_UP, LOW);
      
      N = 0;
  }
  mb.step(db);
  ma.step(da);

  delay(d);
  N += 1;
}
*/

#if 0
/*
  digitalWrite(MOTOR_B1_F,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);
  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_F,  LOW);

  digitalWrite(MOTOR_B2_F,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);
  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_F,  LOW);

  digitalWrite(MOTOR_B1_R,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);
  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_R,  LOW);

  digitalWrite(MOTOR_B2_R,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);
  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_R,  LOW);
*/
  if ((N % 80) < 40) {

  digitalWrite(MOTOR_B1_F,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B2_F,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);
  
  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_F,  LOW);
  delay(d);

  digitalWrite(MOTOR_B1_R,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_F,  LOW);
  delay(d);

  digitalWrite(MOTOR_B2_R,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_R,  LOW);
  delay(d);

  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_R,  LOW);
  delay(d);

  } else {

  digitalWrite(MOTOR_B1_F,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B2_R,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);
  
  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_F,  LOW);
  delay(d);

  digitalWrite(MOTOR_B1_R,  HIGH);
  digitalWrite(MOTOR_B1_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_R,  LOW);
  delay(d);

  digitalWrite(MOTOR_B2_F,  HIGH);
  digitalWrite(MOTOR_B2_EN, HIGH);
  delay(d);

  digitalWrite(MOTOR_B1_EN, LOW);
  digitalWrite(MOTOR_B1_R,  LOW);
  delay(d);

  digitalWrite(MOTOR_B2_EN, LOW);
  digitalWrite(MOTOR_B2_F,  LOW);
  delay(d);
  
  
  }

  N++;
  
}
#endif



