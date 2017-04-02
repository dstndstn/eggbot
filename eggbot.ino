#include <math.h>

#if defined(STANDALONE)
#include "Arduino.h"
#include "avr/io.h"
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

void setup() {
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

float theta = 0.;
int d=1;
float dtheta = 0.01;

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

int lastic = 0;
int lastis = 0;

void loop() {

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
    delay(d);
    theta += dtheta;
}

