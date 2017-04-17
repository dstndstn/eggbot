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

    //Serial.begin(9600);
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


class MotorB {
public:
    MotorB() :
        lastic(0),
        lastis(0),
        theta(0)
    {
    }

    void step(float dtheta) {
        theta += dtheta;
        go_to(theta);
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
    
    void go_to(float t) {
        theta = t;
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

    float theta;

protected:
    int lastic;
    int lastis;
};

MotorB mb;

int ma = 0;

//float theta = M_PI;
int d = 1;
//float dtheta = -0.01;

float theta = 0;
//float dtheta = 0.01;
float DT = 2.*M_PI / 256.0;
float dtheta = DT;

int inited = 0;

float r = 0;
float dr = 2.*M_PI / 80;

int aa = 0;

void loop() {

  if (!inited) {
    /*
      for (int i=0; i<700; i++) {
        mb.step(dtheta);
        delay(d);
      }
    */
    /*
      for (int j=0; j<3; j++) {
        digitalWrite(PEN_DOWN, HIGH);
        delay(50);
        digitalWrite(PEN_DOWN, LOW);

        digitalWrite(PEN_UP, HIGH);
        delay(80);
        digitalWrite(PEN_UP, LOW);
      }
      digitalWrite(PEN_UP, HIGH);
      delay(5000);
      digitalWrite(PEN_UP, LOW);
      digitalWrite(PEN_DOWN, HIGH);
      delay(25);
      digitalWrite(PEN_DOWN, LOW);
      */

     /*
      digitalWrite(PEN_UP, HIGH);
      delay(100);
      for (int i=0; i<5000; i++) {
        mb.step(-dtheta);
        delay(d);
      }
      for (int i=0; i<1500; i++) {
        mb.step(dtheta);
        delay(d);
      }
      digitalWrite(PEN_UP, LOW);
      delay(100);
      */

      mb.theta = 0.;
      mb.go_to(0.);
      mb.theta = 0.;
      delay(1000);

      for (int i=0; i<800; i++) {
        ma++;
        motor_A_to(ma);
        delay(20 * d);
      }


      inited = 1;
  }

  if ((dtheta > 0) && (mb.theta >= 2.*M_PI)) {
      //dtheta *= -1;

      mb.go_to(2.*M_PI);
      delay(100);
      
      for (int i=0; i<800; i++) {
        ma++;
        motor_A_to(ma);
        delay(20 * d);
      }
      digitalWrite(PEN_UP, HIGH);    
      for (;;) {
         delay(1000);
      }
      
      dtheta = -0.1;

      digitalWrite(PEN_UP, HIGH);
      delay(500);

      while (mb.theta > 0) {
          mb.step(dtheta);
          delay(10*d);
      }
      aa = 0;

      digitalWrite(PEN_UP, LOW);
      delay(500);

  //}
  //if ((dtheta < 0) && (mb.theta < 0)) { //-3 * M_PI)) {
      dtheta = DT;
      mb.theta = 0.0 - dtheta;
      mb.go_to(0.0);

  //    digitalWrite(PEN_UP, LOW);
  //    delay(500);

  }


  //if ((aa > 0) && (aa % 8 == 0)) {
      digitalWrite(PEN_UP, HIGH);
      delay(100);
      for (int i=0; i<5; i++) {
        ma++;
        motor_A_to(ma);
        delay(20 * d);
      }
      digitalWrite(PEN_UP, LOW);
      delay(100);

      /*
      for (int i=0; i<64; i++) {
        mb.step(dtheta);
        delay(5*d);
      }
      for (int i=0; i<64; i++) {
        mb.step(-dtheta);
        delay(5*d);
      }
      */
  //}
  aa++;

  mb.step(dtheta);
  delay(10*d);

/*
    float dr = 2.*M_PI / 200;
    for (int i=0; i<10; i++) {
      float s = sin(r);
      r += dr/10.0;
      float t = s * M_PI;
      mb.go_to(t);
      delay(20*d);
    }
*/
    
    ma++;
    motor_A_to(ma);
    delay(20 * d);
}

void loop1() {

    /*if (!inited) {

      for (int i=0; i<5000; i++) {
        mb.step(dtheta);
        delay(d);
      }
      for (int i=0; i<1500; i++) {
        mb.step(-dtheta);
        delay(d);
      }
      inited = 1;
    }
    delay(1000);
    */

    if ((dtheta > 0) && (mb.theta > 2.5 * M_PI)) {
        dtheta *= -1;
    }
    if ((dtheta < 0) && (mb.theta < -3 * M_PI)) {
        dtheta *= -1;
    }

    float t1 = mb.theta;

    for (int i=0; i<10; i++) {
      mb.step(dtheta);
      delay(10*d);
    }

    float t2 = mb.theta;

    if (((t1 > 0) && (t2 < 0)) ||
        ((t1 < 0) && (t2 > 0))) {
          for (int i=0; i<300; i++) {
            mb.step(abs(dtheta));
            delay(d);
          }      
          for (int i=0; i<600; i++) {
            mb.step(-abs(dtheta));
            delay(d);
          }      
          for (int i=0; i<300; i++) {
            mb.step(abs(dtheta));
            delay(d);
          }
      }      

      if ( ((t1 < 2.*M_PI) && (t2 > 2.*M_PI)) ||
           ((t1 > -2.*M_PI) && (t2 < -2.*M_PI)) ) {
            
        for (int i=0; i<300; i++) {
          mb.step(-dtheta);
          delay(d);
        }      
        for (int i=0; i<300; i++) {
          mb.step(dtheta);
          delay(d);
        }      
      }

    
    ma++;
    motor_A_to(ma);
    delay(5 * d);
    ma++;
    motor_A_to(ma);
    delay(5 * d);
    /* laser cal
     if ((dtheta > 0) && (theta > 3.1416)) {
     dtheta *= -1;
     d = 2;
     }
     if ((dtheta < 0) && (theta < 0.)) {
     dtheta *= -1;
     theta = 0.;
     d = 2000;
     }
     */

    /*
    if ((dtheta > 0) && (theta > M_PI * 10)) {
        dtheta *= -1;
    }
    if ((dtheta < 0) && (theta < 0.)) {
        dtheta *= -1;
    }

    //float t = remap_theta(theta);
    float t = theta;
    float s = sin(t);
    float c = cos(t);

    int ic = (int)(255 * c);
    int is = (int)(255 * s);

    int sgn;
    sgn = isign(ic);
    if (isign(lastic) != sgn) {
        digitalWrite(MOTOR_B1_R, LOW);
        digitalWrite(MOTOR_B1_F, LOW);
    }
    sgn = isign(is);
    if (isign(lastis) != sgn) {
        digitalWrite(MOTOR_B2_R, LOW);
        digitalWrite(MOTOR_B2_F, LOW);
    }

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

    delay(d);
    theta += dtheta;
     */

    /*
    if (doserial) {
        Serial.print("ic=");
        Serial.print(ic);
        Serial.print(", is=");
        Serial.print(is);
        Serial.print("   winding 1: F=");
        Serial.print(digitalRead(MOTOR_B1_F));
        Serial.print(", R=");
        Serial.print(digitalRead(MOTOR_B1_R));
        Serial.print("   winding 2: F=");
        Serial.print(digitalRead(MOTOR_B2_F));
        Serial.print(", R=");
        Serial.print(digitalRead(MOTOR_B2_R));
        Serial.println("");
        Serial.flush();
    }
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


