#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

unsigned long ct = 0;   //Temps

MeUltrasonicSensor ultrasound(PORT_10);   //Ultrasound

// // MOTORS ---
//  - Motor Left - from front
const int m1_pwm = 11;
const int m1_in1 = 48; // M1 ENA
const int m1_in2 = 49; // M1 ENB

//  - Motor Right - from front
const int m2_pwm = 10;
const int m2_in1 = 46; // M2 ENA
const int m2_in2 = 47; // M2 ENB

int currentSpeed = 0;
int cruisingSpeed = (255*0.6);
int halfSpeed = (cruisingSpeed*0.5);
// //

//Variables globales
int dist = 0;

enum State {NORMAL, RALENTI, DANGER, RONDE};
State currentState = NORMAL;
const int eightyThreshold = 80;
const int fortyThreshold = 40;
const int celebrateLightParameter = 5;

#define ALL_LEDS 0
#define LEDNUM  12
#define LED_PIN 44
MeRGBLed led( 0, LEDNUM );  //////////////////////////////////////CHECK FOR LEDS PLEAAAAAASE/////////////////////////////////////////////
//

void setup() {
  Serial.begin(115200);
  led.setpin(LED_PIN);
  dist = ultrasound.distanceCm();
}

void loop() {
  ct = millis();
  getDist(ct);

  writeDistAndState(ct);

  switch (currentState) {

    case NORMAL:
      normalState(ct);
      break;
      
    case RALENTI:
      slowState(ct);
      break;

    case DANGER:
      dangerState(ct);
      break;

    case RONDE:
      celebrateState(ct);
      break;
  }
}

void getDist(unsigned long ct) {
  static unsigned long lt;
  const unsigned int waitTime = 175;
  static int lastDistance = 0;
  static int tempDistance;

  if (ct - lt >= waitTime) {

    tempDistance = ultrasound.distanceCm();

    if (tempDistance != 0) {

      dist = tempDistance;
      lastDistance = dist;
    }
    else {

      dist = lastDistance;
    }

    lt = ct;
  }
}

void writeDistAndState(unsigned long ct) {
  static unsigned long lt;
  const unsigned int waitTime = 250;
  static int lastDistance = 0;
  static int tempDistance;

  if (ct - lt >= waitTime) {

    Serial.print("Distance: ");
    Serial.print(dist);
    Serial.println(" cm");

    Serial.print("État: ");

    switch (currentState) {

      case NORMAL:
        Serial.println("Normal");
        break;
      
      case RALENTI:
        Serial.println("Ralenti");
        break;

      case DANGER:
        Serial.println("Danger");
        break;

      case RONDE:
        Serial.println("Célébration! WOO");
        break;
    }

    Serial.println("");
    lt = ct;
  }
}

void normalState(unsigned long ct) {

  static bool firstTime = true;
  static unsigned long timeToCelebrate = 0;

  if (firstTime) {

    firstTime = false;

    timeToCelebrate = ct + 10000;

    currentSpeed = cruisingSpeed;

    //Change ring color to green back half
    lightUp(6, 12, 0, 50, 0); //starting led, ending led, red, green, blue
  }

  forward();

  bool transToSlow = dist < eightyThreshold && dist > fortyThreshold;

  bool transToDanger = dist < fortyThreshold;

  bool transToCelebrate = ct > timeToCelebrate;

  if (transToSlow) {

    firstTime = true;

    currentState = RALENTI;

    return;
  }

  if (transToDanger) {

    firstTime = true;

    currentState = DANGER;

    return;
  }

  if (transToCelebrate) {

    firstTime = true;

    currentState = RONDE;

    return;
  }
}

void slowState(unsigned long ct) {

  static bool firstTime = true;

  if (firstTime) {

    firstTime = false;

    currentSpeed = halfSpeed;

    //Change ring color to blue front half
    lightUp(12, 6, 0, 0, 50); //starting led, ending led, red, green, blue
  }

  forward();

  bool transToNormal = dist >= eightyThreshold;

  bool transToDanger = dist < fortyThreshold;

  if (transToNormal) {
    currentState = NORMAL;

    firstTime = true;
    return;
  }

  if (transToDanger) {
    currentState = DANGER;

    firstTime = true;
    return;
  }
}

void dangerState(unsigned long ct) {

  static bool firstTime = true;

  if (firstTime) {

    firstTime = false;

    stop();

    currentSpeed = cruisingSpeed;

    //Change ring color to red
    lightUp(1, 12, 50, 0, 0); //starting led, ending led, red, green, blue
  }

  bool transToNormal = inDanger(ct);

  if (transToNormal) {
    currentState = NORMAL;

    firstTime = true;
    return;
  }
}

void celebrateState(unsigned long ct) {

  static bool firstTime = true;

  if (firstTime) {

    firstTime = false;

    currentSpeed = cruisingSpeed;

    stop();

    //Turn off lights
    led.setColor (0, 0, 0);
    led.show();
  }

  bool transToNormal = celebrateTask(ct);

  if (transToNormal) {
    currentState = NORMAL;

    firstTime = true;
    return;
  }
}

bool inDanger(unsigned long ct) {

  static unsigned long lt = ct;
  static unsigned long waitTime = 500;
  static unsigned long backTime = 1000;
  static unsigned long turnLeftTime = 500;
  static bool firstTime = true;
  static bool isWaiting = false;
  static bool isGoingBackwards = false;
  static bool isTurning = false;

  if (firstTime) {

    firstTime = false;

    lt = ct;

    isWaiting = true;
  }

  if (isWaiting) {

    if (ct - lt >= waitTime) {

      lt = ct;

      isWaiting = false;

      isGoingBackwards = true;
    }
  }

  if (isGoingBackwards) {

    backwards();

    if (ct - lt >= backTime) {

      lt = ct;

      isGoingBackwards = false;

      isTurning = true;
    }
  }

  if (isTurning) {

    turnLeft();

    if (ct - lt >= turnLeftTime) {

      firstTime = true;

      isTurning = false;

      return true;
    }
  }

  return false;
}

bool celebrateTask(unsigned long ct) {

  static bool firstTime = true;
  const unsigned int exitTime = 2000;
  const unsigned int waitTime = 250;
  static unsigned long lt = ct;
  static unsigned long goneIn = ct;
  static int startLed = 2;
  static int endLed = 4;

  if (firstTime) {

    firstTime = false;

    goneIn = ct;
  }

  if (ct - lt >= waitTime) {

    startLed += celebrateLightParameter;
    endLed += celebrateLightParameter;

    if (startLed > 12) {startLed -= 12;}
    if (endLed > 12) {endLed -= 12;}


    lightUp(startLed, endLed, 0, 100, 0); //starting led, ending led, red, green, blue
    lt = ct;
  }

  if (ct - goneIn >= exitTime) {

    firstTime = true;

    return true;
  }
  else {

    return false;
  }
}

void lightUp(int startLed, int endLed, int r, int g, int b) {

  led.setColor(0, 0, 0);   //All off

  if (startLed > endLed) {

    led.setColor(0, r, g, b);

    for (int i = (endLed + 1); i <= (startLed - 1); i++) {
      
      led.setColor(i, 0, 0, 0);
    }
  }
  else {

    for (int i = startLed; i <= endLed; i++) {
      
      led.setColor(i, r, g, b);
    }
  }

  led.show();
}

void forward() {

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, currentSpeed);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, currentSpeed);
}

void backwards() {

  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, currentSpeed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, currentSpeed);
}

void turnLeft() {

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, currentSpeed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, currentSpeed);
}

void stop() {

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, currentSpeed);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, currentSpeed);
}