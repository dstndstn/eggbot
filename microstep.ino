#include <math.h>

/*
#define STEPA1 2
#define STEPA2 3
#define STEPA3 4
#define STEPA4 5

#define STEPB1 8
#define STEPB2 9
#define STEPB3 10
#define STEPB4 11
*/

/**
 * Old setup:
 * 8  -> M2 reverse
 * 9  -> M1 forward
 * 10 -> M2 forward
 * 11 -> M1 reverse
 *
 * 2 -> A1 
 * 3 -> A2
 * 4 -> A3
 * 5 -> A4
 * 
 *
 * New setup:
 * 
 * 11 -> M1 enable
 *  3 -> M2 enable
 *
 * 2 -> A1
 * 4 -> A2
 * 5 -> A3
 * 6 -> A4
 * 
 * 7 -> M2 reverse
 * 8 -> M1 forward
 * 9 -> M2 forward
 * 10-> M1 reverse
 */

#define STEPA1 2
#define STEPA2 4
#define STEPA3 5
#define STEPA4 6

#define MB2R 7
#define MB1F 8
#define MB2F 9
#define MB1R 10

#define MB1EN 11
#define MB2EN  3

#define STEPB1 7
#define STEPB2 8
#define STEPB3 9
#define STEPB4 10

#define PWMB1 3
#define PWMB2 11

/**
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
   pinMode(STEPA1, OUTPUT);
   pinMode(STEPA2, OUTPUT);
   pinMode(STEPA3, OUTPUT);
   pinMode(STEPA4, OUTPUT);

   pinMode(STEPB1, OUTPUT);
   pinMode(STEPB2, OUTPUT);
   pinMode(STEPB3, OUTPUT);
   pinMode(STEPB4, OUTPUT);

   pinMode(PWMB1, OUTPUT);
   pinMode(PWMB2, OUTPUT);

  digitalWrite(STEPA1, LOW);
  digitalWrite(STEPA2, LOW);
  digitalWrite(STEPA3, LOW);
  digitalWrite(STEPA4, LOW);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, LOW);

  // Timer 2: WMG2 = 0x3 (fast PWM), COM2[AB] = 0x2 (non-inverting PWM)
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);

  // Timer 2: WMG2 = 0x1 (phase correct PWM), COM2[AB] = 0x2 (non-inverting PWM)
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);

  // Timer 2: prescaler = 1 (CS2 = 0x1)
  TCCR2B = _BV(CS20);

  // Timer 2: prescaler = 8 (CA22=0, CA21=1, CS20=0)
  //TCCR2B = _BV(CS21);
  // audible!

  // Timer 2: prescaler = 32
  //TCCR2B = _BV(CS21) | _BV(CS20);
  // audible!

  // Timer 2: prescaler = 256
  //TCCR2B = _BV(CS22) | _BV(CS21);

  // Timer 2: prescaler = 1024
  //TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
  // vibrates!

  OCR2A = 0;
  OCR2B = 0;

  Serial.begin(9600);
}



int sine[] = {
       0,   6,  12,  18,  25,  31,  37,  44,
      50,  56,  62,  68,  74,  80,  86,  92,
      98, 104, 110, 115, 121, 127, 132, 137,
     143, 148, 153, 158, 163, 168, 173, 177,
     182, 186, 190, 194, 199, 202, 206, 210,
     213, 217, 220, 223, 226, 229, 232, 234,
     237, 239, 241, 243, 245, 247, 248, 250,
     251, 252, 253, 254, 255, 255, 255, 255,
};

void rampAtoB(int d) {
  int i,j;
  int N = sizeof(sine)/sizeof(int);
  for (i=0, j=N-1; i<N; i++, j--) {
    OCR2A = sine[i];
    OCR2B = sine[j];
    delay(d);
  }
}

void rampBtoA(int d) {
  int i,j;
  int N = sizeof(sine)/sizeof(int);
  for (i=0, j=N-1; i<N; i++, j--) {
    OCR2B = sine[i];
    OCR2A = sine[j];
    delay(d);
  }
}

float theta = 0.;

//const int d = 100;
//float dtheta = 0.03;

//const int d = 2000;
// pi/4
//float dtheta = 0.785398;
// pi/8
//float dtheta = 0.3927;

//int d = 10;
//float dtheta = 0.01;
//float dtheta = 1.571 / 20;
//float dtheta = 0.0785398163397448; // pi/40.
int d=1;
float dtheta = 0.01;

int doserial = 0;

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


/*
  if (doserial) {
    Serial.print("Theta ");
    Serial.print(theta);
    Serial.print("  ");
  }
  if ((dtheta > 0) && (theta > 3*6.3)) {
      if (doserial)
        Serial.println("Change direction!");
      dtheta *= -1;
  }
  if ((dtheta < 0) && (theta < 0.)) {
      if (doserial)
        Serial.println("Change direction!");
      dtheta *= -1;
  }
*/  
  //float t = remap_theta(theta);
  float t = -theta;
  float s = sin(t);
  float c = cos(t);
  //float s = sin(theta);
  //float c = cos(theta);

  //OCR2B = int(abs(255 * s));
  //OCR2A = int(abs(255 * c));
  int ic = int(255 * c);
  int is = int(255 * s);
  analogWrite(MB2EN, abs(is));
  analogWrite(MB1EN, abs(ic)); //int(abs(255 * c)));

  // windings direction enable bits
  int sgn = isign(ic);
  if (isign(lastic) != sgn) {
    if (sgn == 0) {
      digitalWrite(MB1R, LOW);
      digitalWrite(MB1F, LOW);
    } else if (sgn > 0) {
      digitalWrite(MB1R, LOW);
      digitalWrite(MB1F, HIGH);
    } else {
      digitalWrite(MB1F, LOW);
      digitalWrite(MB1R, HIGH);
    }
  }
  lastic = ic;

  sgn = isign(is);
  if (isign(lastis) != sgn) {
    if (sgn == 0) {
      digitalWrite(MB2R, LOW);
      digitalWrite(MB2F, LOW);
    } else if (sgn > 0) {
      digitalWrite(MB2R, LOW);
      digitalWrite(MB2F, HIGH);
    } else {
      digitalWrite(MB2F, LOW);
      digitalWrite(MB2R, HIGH);
    }
  }
  lastis = is;
  
  if (doserial) {
    Serial.print("ic=");
    Serial.print(ic);
    Serial.print(", is=");
    Serial.print(is);
    Serial.print("   winding 1: F=");
    Serial.print(digitalRead(MB1F));
    Serial.print(", R=");
    Serial.print(digitalRead(MB1R));
    Serial.print("   winding 2: F=");
    Serial.print(digitalRead(MB2F));
    Serial.print(", R=");
    Serial.print(digitalRead(MB2R));
    Serial.println("");
    Serial.flush();
  }
  delay(d);
  theta += dtheta;

}


void loopy() {
  theta += dtheta;

  if ((dtheta > 0) && (theta > 3.1)) {
      dtheta *= -1;
  }
  if ((dtheta < 0) && (theta < 0.)) {
      dtheta *= -1;
  }
  
  float s = sin(theta);
  float c = cos(theta);
  //OCR2B = int(abs(255 * s));
  //OCR2A = int(abs(255 * c));
  int ic = int(255 * c);
  int is = int(255 * s);
  analogWrite(MB2EN, abs(is));
  analogWrite(MB1EN, abs(ic)); //int(abs(255 * c)));

  // windings direction enable bits
  if (ic <= 0) {
    digitalWrite(MB1R, LOW);
    digitalWrite(MB1F, HIGH);
  } else {
    digitalWrite(MB1F, LOW);
    digitalWrite(MB1R, HIGH);
  }

  if (is >= 0) {
    digitalWrite(MB2R, LOW);
    digitalWrite(MB2F, HIGH);
  } else {
    digitalWrite(MB2F, LOW);
    digitalWrite(MB2R, HIGH);
  }

/*
  Serial.print("c=");
  Serial.print(c);
  Serial.print(", s=");
  Serial.print(s);
  Serial.print(", ic=");
  Serial.print(ic); 
  Serial.print(", is=");
  Serial.print(is);
  Serial.print(", abs ic=");
  Serial.print(abs(ic));
  Serial.print(", abs is=");
  Serial.print(abs(is));
  Serial.println("");
  */
  Serial.print("ic=");
  Serial.print(ic);
  Serial.print(", is=");
  Serial.print(is);
  Serial.print("   winding 1: F=");
  Serial.print(digitalRead(MB1F));
  Serial.print(", R=");
  Serial.print(digitalRead(MB1R));
  Serial.print("   winding 2: F=");
  Serial.print(digitalRead(MB2F));
  Serial.print(", R=");
  Serial.print(digitalRead(MB2R));
  Serial.println("");
  Serial.flush();
  delay(d);

/*
  int d = 1;
  forward(d);
  forward(d);
  forward(d);
  forward(d);

  reverse(d);
  reverse(d);
  reverse(d);
  reverse(d);
  */
}

void reverse(int d) {
  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, HIGH);

  rampAtoB(d);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, HIGH);

  rampBtoA(d);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, LOW);

  rampAtoB(d);

  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, LOW);

  rampBtoA(d);
  
}


void forward(int d) {
  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, LOW);

  //OCR2B = 255;
  rampAtoB(d);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, LOW);

  //OCR2A = 255;
  rampBtoA(d);
  
  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, HIGH);

  //OCR2B = 255;
  rampAtoB(d);

  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, HIGH);

  //OCR2A = 255;
  rampBtoA(d);
}




void rampUpA(int d) {
  int i;
  for (i=0; i<256; i++) {
    OCR2A = i;
    delay(d);
  }
}

void rampDownA(int d) {
  int i;
  for (i=255; i>=0; i--) {
    OCR2A = i;
    delay(d);
  }
}

void rampUpB(int d) {
  int i;
  for (i=0; i<256; i++) {
    OCR2B = i;
    delay(d);
  }
}

void rampDownB(int d) {
  int i;
  for (i=255; i>=0; i--) {
    OCR2B = i;
    delay(d);
  }
}
void loop__SQUARE() {

  int d = 1;

  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, LOW);

  OCR2B = 255;
  rampUpA(d);
  rampDownB(d);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, HIGH);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, LOW);

  OCR2A = 255;
  rampUpB(d);
  rampDownA(d);

  digitalWrite(STEPB1, LOW);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, HIGH);
  digitalWrite(STEPB4, HIGH);

  OCR2B = 255;
  rampUpA(d);
  rampDownB(d);

  digitalWrite(STEPB1, HIGH);
  digitalWrite(STEPB2, LOW);
  digitalWrite(STEPB3, LOW);
  digitalWrite(STEPB4, HIGH);

  OCR2A = 255;
  rampUpB(d);
  rampDownA(d);

  delay(1000);

}
