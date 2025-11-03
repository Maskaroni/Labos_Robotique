// Milanne Lacerte || Deniers chiffres DA: 39
// Mon robot fait donc 90° vers la droite et a une del bleu qui tourne antihoraire lors de son retour.

#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

unsigned long ct = 0;   //Temps

MeUltrasonicSensor ultrasound(PORT_10);   //Ultrasound
MeGyro gyro(0, 0x69);   // Gyroscope

// Distance variables for presentation
float pathX = 2.1;
float pathY = 50;
short int numberOfPivot = 1;
short int bufferStop = 200;
//

#pragma region - Global variables
int dist = 0;
int angle = 0;
short int cruisingSpeed = 100;
short int pivotSpeed = 58;

enum State {SETUP, FORWARD, PIVOTING, DELIVERING};
State currentState = SETUP;

#define ALL_LEDS 0
#define LEDNUM  12
#define LED_PIN 44
MeRGBLed led( 0, LEDNUM );
#pragma endregion

#pragma region - Variables for encoder
#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9.0
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
#define PULSE_PER_METER ((365.0/DIA_WHEEL) * (PULSE * RATIO))
#define PULSE_HALF_SPIN (((FULL_SPIN_CIRC/2.0)/CIRC_WHEEL)*PULSE*RATIO)

long goal = (pathX * PULSE_PER_METER);

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
#pragma endregion

#pragma region - Encoder config
// ********* INTERRUPTIONS ***********

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void) {
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
    encoderLeft.pulsePosPlus();
  }
}

// ************* DÉBUT ************

void encoderConfig() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);
  
  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);
  
  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
  
  // DÉBUT : Ne pas modifier ce code!
  // Configuration de la fréquence du PWM
  // Copier-coller ce code si on désire
  // travailler avec les encodeurs
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  // FIN : Ne pas modifier ce code!
}
#pragma endregion

void setup() {

  Serial.begin(115200);

  encoderConfig();
  encoderRight.setPulsePos(0);
  encoderLeft.setPulsePos(0);

  gyro.begin();

  led.setpin(LED_PIN);
  led.setColor(0, 0, 0, 0);

  dist = ultrasound.distanceCm();

}

void goStraight(short speed = 100, short firstRun = 0) {
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // PD Controller
    // Change les valeurs selon tes besoins
    // higher kp = plus réactive, peu osciller
    // lowewr kp = sluggish, moins d'oscillation
    // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 10.75;
    //const double ki = 1.0;
    const double kd = 5.0;
    
    if (firstRun) {
      firstRun = 0;

      zAngleGoal = gyro.getAngleZ();
      
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    // Google : ELI5 PID
    // Astuce web : ELI5 = Explain Like I'm 5
    output = kp * error + kd * (error - previousError);
    
    previousError = error;
    
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);
}

void pivotState(bool changeFirstTime = false) {
  static bool firstTime = true;

  if (changeFirstTime) {
    firstTime = !firstTime;
  }

  if (firstTime) {

    firstTime = false;

    pivotTask(pivotSpeed, 1);

    lightUp(1, 12, 0, 0, 0); //starting led, ending led, red, green, blue
  }

  pivotTask(pivotSpeed, 0);
  bool transToForward = numberOfPivot < 1;

  if (transToForward) {
    currentState = FORWARD;

    goStraight(cruisingSpeed, 1);

    numberOfPivot = 1;
    firstTime = true;
    return;
  }
}

void loop() {

  ct = millis();
  dist = getDist(ct);

  switch (currentState) {

    case SETUP:
      setupState(ct);
      break;
      
    case FORWARD:
      forwardState();
      break;

    case PIVOTING:
      pivotState();
      break;

    case DELIVERING:
      deliveryState(ct);
      break;
  }

  gyro.update();
  encodersUpdate();
}

void encodersUpdate() {
  encoderRight.loop();
  encoderLeft.loop();
}

int getDist(unsigned long ct) {
  static unsigned long lt;
  const unsigned int waitTime = 175;
  static int lastDistance = 0;
  static int tempDistance = 0;
  static int goodDistance = 0;


  if (ct - lt >= waitTime) {

    tempDistance = ultrasound.distanceCm();

    if (tempDistance != 0) {

      goodDistance = tempDistance;
      lastDistance = tempDistance;

    }
    else {

      goodDistance = lastDistance;

    }

    lt = ct;
    return goodDistance;
  }
}

void pivotTask(short speed, short firstRun) {
  static double zAngleGoal = 0.0;
    
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;
    
  // PD Controller
  // Change les valeurs selon tes besoins
  // higher kp = plus réactive, peu osciller
  // lowewr kp = sluggish, moins d'oscillation
  // higher kd = limite l'oscillation, la bonne valeur arrête l'oscillation
  const double kp = 2.1;
  //const double ki = 1.0;
  const double kd = 2.1;
    
  if (firstRun) {
    firstRun = 0;

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);

    gyro.resetData();

    double tempAngle = gyro.getAngleZ();
    zAngleGoal = tempAngle + 90;
      
    return;
  }
    
  error = gyro.getAngleZ() - zAngleGoal;
    
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);
    
  previousError = error;
    
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(speed - output);             // CHECKER SI +speed + output

  if (gyro.getAngleZ() >= zAngleGoal) {

    numberOfPivot = numberOfPivot - 1;
    return;
  }
}

void setupState(unsigned long ct) {

  bool transToForward = waitThreeSecTask(ct);

  if (transToForward) {

    currentState = FORWARD;
    goStraight(cruisingSpeed, 1);

    static unsigned long lt = ct;
    return;
  }
}

void forwardState() {

  static bool firstTime = true;
  static bool firstPathDone = false;
  static bool secondPathDone = false;
  static double posBeforeSecPath = 0;

  if (firstTime) {

    firstTime = false;

    if (!firstPathDone) {
      encoderRight.moveTo(goal, cruisingSpeed);
      encoderLeft.moveTo(goal, cruisingSpeed);
    }

    if (firstPathDone && !secondPathDone) {
      posBeforeSecPath = encoderLeft.getCurPos();
    }

    if (firstPathDone && secondPathDone) {
      encoderRight.moveTo(posBeforeSecPath, cruisingSpeed);
      encoderLeft.moveTo(posBeforeSecPath, cruisingSpeed);
    }
  }

  if (!firstPathDone && !secondPathDone) { firstPathLights(); }
  if (firstPathDone && secondPathDone) { afterDeliveryLights(); }
  goStraight(cruisingSpeed, 0);
  

  if (!secondPathDone) {

    bool transition = firstPathDone ? dist <= pathY : (encoderLeft.distanceToGo() <= bufferStop);

    if (transition && firstPathDone) {

      firstTime = true;

      secondPathDone = true;

      currentState = DELIVERING;

      lightUp(1, 12, 0, 0, 0);

      return;
    }

    if (transition && !firstPathDone) {

      firstTime = true;

      currentState = PIVOTING;

      firstPathDone = true;
      return;
    }
  }

  bool transEnd = (firstPathDone && secondPathDone) ? encoderLeft.distanceToGo() >= 0 : false;

  if (transEnd && firstPathDone && secondPathDone) {

    bool firstStop = true;
    firstTime = true;

    if (firstStop) {
      stop();
      lightUp(1, 12, 30, 20, 3);

      firstStop = false;
    }

    return;
  }
}

void deliveryState(unsigned long ct) {

  deliveryLights(ct);
  bool transition = waitThreeSecTask(ct);

  if (transition) {

    cruisingSpeed = (cruisingSpeed) * (-1);
    currentState = FORWARD;

    return;
  }
}

void firstPathLights() {

  int endlight = map(encoderLeft.distanceToGo(), goal, 0, 1, 12);
  Serial.println(encoderLeft.distanceToGo());
  Serial.println(endlight);

  lightUp(1, endlight, 30, 20, 1); //starting led, ending led, red, green, blue
}

void afterDeliveryLights() {

  const unsigned int waitTime = 300;
  static unsigned long lt = ct;
  static int led = 1;

  if (ct - lt >= waitTime) {

    led = led - 1;

    if (led <= 0) {led += 12;}

    lightUp(led, led, 0, 0, 200); //starting led, ending led, red, green, blue
    lt = ct;
  }
}

void deliveryLights(unsigned long ct) {

  const unsigned int waitTime = 250;
  static unsigned long lt = ct;
  static bool lightsOn = true;

  if (ct - lt >= waitTime) {
    lightsOn = !lightsOn;

    if (lightsOn) {

      lightUp(1, 12, 0, 100, 0); //starting led, ending led, red, green, blue
    }
    else {

      lightUp(1, 12, 0, 0, 0); //starting led, ending led, red, green, blue
    }
    lt = ct;
  }
}

bool waitThreeSecTask(unsigned long ct) {

  static bool firstTime = true;
  const unsigned int exitTime = 2000;
  static unsigned long goneIn = ct;

  if (firstTime) {

    firstTime = false;

    goneIn = ct;

    stop();
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

void stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  encoderLeft.move(0);
  encoderRight.move(0);
}
