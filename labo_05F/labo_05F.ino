#include <MeAuriga.h>
#include <ezBuzzer.h>
#include <Adafruit_seesaw.h>
//#include json
//#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

unsigned long ct = 0;   // Temps
int dist = 0;
MeGyro gyro(0, 0x69);   // Gyroscope
MeUltrasonicSensor ultrasound(PORT_10);   //Ultrasound

Adafruit_seesaw sees;   // Seesaw

#pragma region - Global variables ------------

bool firstForward = true;
const int thirdyThreshold = 30;
const int twentyThreshold = 20;

double zAngleGoal = 0.0;
long position = 0.0;
long leftPosition = 255;  //max 2000
long rightPosition = -255;  //max -2000

short int leftSpeed = 100;
short int rightSpeed = 100;
long minSpeed = 50;
long maxSpeed = 80;   // Speed variables

enum State {CHECK_FOR_LINE, GO, DANGER, PIVOT};
State currentState = CHECK_FOR_LINE; // States

#define ALL_LEDS 0
#define LEDNUM  12
#define LED_PIN 44
MeRGBLed led( 0, LEDNUM );  // Lumières

#define BUZZER_PIN 45
ezBuzzer buzzer(BUZZER_PIN);  // Buzzer

int melody[] = {NOTE_E5};
int duration[] = {10};  // Son produit

const short nbrCapteurs = 5;
struct Capteurs {
  int valMin = 1023;
  int valMax = 0;
  int newVal;
  int valNormed;
  int seuil;
  byte isOnLine;
};
Capteurs capteurs[nbrCapteurs];  // Capteurs et leurs valeurs associées

#pragma endregion

#pragma region - Encoder stuff ------------

#define DIST_WHEEL 151
#define DIA_WHEEL 64.5
#define PULSE 9.0
#define RATIO 39.267
#define FULL_TURN_CIRC 948.8
#define FULL_SPIN_CIRC 474.4
#define CIRC_WHEEL 202.6
#define PULSE_PER_METER ((365.0/DIA_WHEEL) * (PULSE * RATIO))
#define PULSE_HALF_SPIN (((FULL_SPIN_CIRC/2.0)/CIRC_WHEEL)*PULSE*RATIO)

MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);

void rightEncoderInterrupt(void) {
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
  
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

#pragma endregion


void setup() {

  Serial.begin(115200);

  if (!sees.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }

  Serial.println("Connexion réussie au LyneTracker!");

  encoderConfig();
  encoderRight.setPulsePos(0);
  encoderLeft.setPulsePos(0);

  gyro.begin();

  led.setpin(LED_PIN);
  led.setColor(0, 0, 0, 0);
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
  const double kp = 1.75;
  //const double ki = 1.0;
  const double kd = 1.0;
    
  if (firstRun) {
    firstRun = 0;

    gyro.resetData();

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

void pivot(short speed, short firstRun = 0, int goToAngle = 0) {
    
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

    gyro.resetData();

    double tempAngle = gyro.getAngleZ();
    zAngleGoal = tempAngle + goToAngle;

    encoderLeft.setMotorPwm(speed);
    encoderRight.setMotorPwm(speed);
      
    return;
  }
    
  error = gyro.getAngleZ() - zAngleGoal;

    
  // Google : ELI5 PID
  // Astuce web : ELI5 = Explain Like I'm 5
  output = kp * error + kd * (error - previousError);
    
  previousError = error;
    
  encoderLeft.setMotorPwm(speed - output);
  encoderRight.setMotorPwm(speed - output);
}

void loop() {

  ct = millis();
  dist = getDist(ct);

  isTouchingLine();
  getPosition();

  Serial.print("P ");
  Serial.println(position);

  switch (currentState) {

    case CHECK_FOR_LINE:
      checkForLine();
      break;
    case GO:
      going(ct);
      break;
      
    case DANGER:
      slowing();
      break;

    case PIVOT:
      pivoting();
      break;
  }

  gyro.update();
  encodersUpdate();

  // if (sensorValues[2] < ) {
  //   // Le capteur du milieu voit la ligne, avancer
  //   Forward();
  // }
  // else if (sensorValues[0] < capteurs[0].seuil || sensorValues[1] < capteurs[1].seuil) {
  //   // Les capteurs de gauche voient la ligne, tourner à gauche
  //   TurnLeft();
  // }
  // else if (sensorValues[3] < capteurs[3].seuil || sensorValues[4] < capteurs[4].seuil) {
  //   // Les capteurs de droite voient la ligne, tourner à droite
  //   TurnRight();
  // }
  // else {
  //   // Aucun capteur ne voit la ligne, arrêter ou chercher la ligne
  //   Stop();
  // }
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

short isTouchingLine() {
  for (int i = 0; i < nbrCapteurs; i++) {

    capteurs[i].newVal = sees.analogRead(i);
    capteurs[i].valNormed = ((capteurs[i].newVal - capteurs[i].valMin) * 1.0 / (capteurs[i].valMax - capteurs[i].valMin) * 1000.0);

    if (100 < capteurs[i].valNormed) {

      capteurs[i].isOnLine = 1;
    }
  }
}

int getPosition() {

  double nume = 0.0;
  double denom = 0.0;

  for (int i = 0; i < nbrCapteurs; i++) {

    nume += capteurs[i].valNormed * (i - 2);
    denom += capteurs[i].valNormed;
  }

  return ((nume / denom) * 1000);
}

void calibratePos(float pos) {

  
  if (pos < rightPosition) {

    rightPosition = pos;
  }

  if (pos > leftPosition) {

    leftPosition = pos;
  }

  position = pos;
}

// void TurnRight() {
//   encoderLeft.setMotorPwm(cruisingSpeed);
//   encoderRight.setMotorPwm(cruisingSpeed);
// }

// void TurnLeft() {
//   encoderLeft.setMotorPwm(-cruisingSpeed);
//   encoderRight.setMotorPwm(-cruisingSpeed);
// }

void checkForLine() {

  Serial.println("Checking time");

  static bool firstTime = true;

  if (firstTime) {

    goStraight(maxSpeed/2, 1);

    firstTime = false;
  }

  goStraight(maxSpeed/2, 0);

  bool transToGo = false;

  for (int i = 0; i < nbrCapteurs; i++) {

    if (capteurs[i].isOnLine > 0) {
      transToGo = true;   //pour ne pas qu'il retourne à false si le prochaine n'est pas sur la ligne
    }
  }

  if (transToGo) {
    currentState = GO;

    firstTime = true;
    return;
  }
}

void going(unsigned long ct) {

  Serial.println("Go time");

  static bool firstTime = true;

  if (firstTime) {


    firstTime = false;
  }

  go();

  bool transToDanger = dist <= thirdyThreshold;

  bool transToPivot = dist <= twentyThreshold;

  if (transToDanger) {
    currentState = DANGER;

    firstTime = true;
    return;
  }

  if (transToPivot) {
    currentState = PIVOT;

    firstTime = true;
    return;
  }
}

void slowing() {

  Serial.println("Slow time");

  static bool firstTime = true;

  if (firstTime) {

    goStraight(minSpeed, 1);

    firstTime = false;
  }

  goStraight(minSpeed, 0);

  bool transToGo = true;
  if ((position > 2000) || (position < -2000)) {

    transToGo = false;
  }

  bool transToPivot = dist <= twentyThreshold;

  if (transToGo) {
    currentState = GO;

    firstTime = true;
    return;
  }

  if (transToPivot) {
    currentState = PIVOT;

    firstTime = true;
    return;
  }
}

void pivoting() {

  Serial.println("Pivot time");

  static bool firstTime = true;

  if (firstTime) {

    pivot(minSpeed, 1, 100);
    firstTime = false;
  }

  pivot(minSpeed, 0);

  if (gyro.getAngleZ() >= (zAngleGoal-1) || gyro.getAngleZ() <= ((-zAngleGoal)+1)) {

    currentState = DANGER;
    firstTime = true;
    return;
  }
}

void go() {

  // Ajuster la direction en fonction des valeurs
  leftSpeed = map(position, rightPosition, leftPosition, minSpeed, maxSpeed);
  rightSpeed = map(position, leftPosition, rightPosition, minSpeed, maxSpeed);

  Serial.print("L ");
  Serial.println(leftSpeed);
  Serial.print("RI ");
  Serial.println(rightSpeed);


  encoderLeft.setMotorPwm(leftSpeed);
  encoderRight.setMotorPwm(-rightSpeed);
}

void stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  encoderLeft.move(0);
  encoderRight.move(0);
}