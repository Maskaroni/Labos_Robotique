// Milanne Lacerte || DA: 2486739
// Mon robot tourne à gauche en premier et allume l'avant pour l'état du colis.

#include <MeAuriga.h>
#include <ezBuzzer.h>
#include <Adafruit_seesaw.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

unsigned long ctime = 0;   // Temps
int dist = 0;
MeGyro gyro(0, 0x69);   // Gyroscope
MeUltrasonicSensor ultrasound(PORT_10);   //Ultrasound
Adafruit_seesaw sees;   // Seesaw

#pragma region - Global variables ------------

enum StateMan {WAIT, FORWARD, BACKWARDS, TURNING_RIGHT, TURNING_LEFT, CELEBRATE};
StateMan ct = WAIT;

bool hasGivenPackage = false;
bool hasPackage = false;
static short checkpoint = 1;
const int thirdyThreshold = 30;
const int twentyThreshold = 20;
String serialMsg = "";
bool debugOn = false;

double zAngleGoal = 0.0;
long position = 0.0;
long leftPosition = 255;  //max 2000
long rightPosition = -255;  //max -2000

short int leftSpeed = 100;
short int rightSpeed = 100;
long minSpeed = 60;
long maxSpeed = 180;   
const int calibratingSpeed = 70; // Speed variables

enum State {MANUAL, CALIBRATE, COUNTDOWN, GO_TO_LINE, 
  PACKAGE_SEEKING, FOLLOW_LINE, LOOK_T, PARK};
State currentState = MANUAL; // States

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

#pragma region - Forward and Pivot ----
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
#pragma endregion

void loop() {
  ctime = millis();
  dist = getDist(ctime);

  switch (currentState) {

    case MANUAL:
      Manual();
      break;
    case CALIBRATE:
      Calibrating();
      break;
      
    case COUNTDOWN:
      CountingDown();
      break;

    case GO_TO_LINE:
      GoToLine();
      break;

    case PACKAGE_SEEKING:
      GettingPackage();
      break;

    case FOLLOW_LINE:
      FollowLine();
      break;

    case LOOK_T:
      Crossing();
      break;

    case PARK:
      Parking();
      break;
  }

  buzzer.loop();
  gyro.update();
  encodersUpdate();
}

void encodersUpdate() {
  encoderRight.loop();
  encoderLeft.loop();
}

#pragma region - Manual ----
void serialEvent() {

  static String lastCmd = "no";

  if (!Serial.available()) return;
  
  serialMsg = Serial.readStringUntil('\n');

  if (serialMsg.length() > 2) {

    // Romoving the "0xFF55"
    serialMsg.remove(0, 2);
  }

  if (debugOn) {
    Serial.print ("Le robot a reçu : ");
    Serial.println(serialMsg);
  }

  //separating the parts

  String cmd = "";
  String secondArg = "";
  String r = "";
  String g = "";
  String b = "";

  int firstComma = serialMsg.indexOf(',');
  int secondComma = serialMsg.indexOf(',', firstComma + 1);
  int thirdComma = serialMsg.indexOf(',', secondComma + 1);
  int fourthComma = serialMsg.indexOf(',', thirdComma + 1);

  if (lastCmd == serialMsg) return;

  if (firstComma == -1) {

    cmd = serialMsg;

    processSmallCommand(cmd);
  }
  else {

    cmd = serialMsg.substring(0, firstComma);

    if (secondComma == -1) {

      secondArg = serialMsg.substring(firstComma + 1);

      if (cmd == "p") {

        changeSpeed(secondArg.toInt());
      }
    } 
    else if (fourthComma == -1) {

      r = serialMsg.substring(firstComma + 1, secondComma);
      g = serialMsg.substring(secondComma + 1, thirdComma);
      b = serialMsg.substring(thirdComma + 1);

      lightUp(1, 12, r.toInt(), g.toInt(), b.toInt());
    }
    else {

      secondArg = serialMsg.substring(firstComma + 1, secondComma);
      r = serialMsg.substring(secondComma + 1, thirdComma);
      g = serialMsg.substring(thirdComma + 1, fourthComma);
      b = serialMsg.substring(fourthComma + 1);

      lightUp(secondArg.toInt(), secondArg.toInt(), r.toInt(), g.toInt(), b.toInt());
    }
  }

  lastCmd = serialMsg;
}

void processSmallCommand(String cmd) {

  static bool lights = false;

  if (!lights) {

    lightUp(1, 12, 0, 0, 0);
  }

  if (cmd == "F") {

    goStraight(maxSpeed, 1);
    ct = FORWARD;
  }
  else if (cmd == "B") {
    
    goStraight(-maxSpeed, 1);
    ct = BACKWARDS;
  }
  else if (cmd == "R") {
  
    ct = TURNING_RIGHT;
  }
  else if (cmd == "L") {

    ct = TURNING_LEFT;
  }
  else if (cmd == "S") {

    buzzer.stop();
    stop();
    ct = WAIT;
  }
  else if (cmd == "C") {
    Serial.write("C");
    stop();
    ct = WAIT;
    currentState = CALIBRATE;
  }
  else if (cmd == "O") {
    Serial.write("O");
    stop();
    ct = WAIT;
    currentState = FOLLOW_LINE;
  }
  else if (cmd == "K") {

    if (buzzer.getState() == BUZZER_IDLE) {
      int length = sizeof(duration) / sizeof(int);
      buzzer.playMelody(melody, duration, length);
    }
  }
  else if (cmd == "LIGHT_TOGGLE") {

    lights = !lights;

    if (lights) {

      lightUp(1, 12, 60, 60, 60);
    }
    else {

      lightUp(1, 12, 0, 0, 0);
    }
  }
  else {

    Serial.println("Mauvaise commande");
  }
}

void Manual() {
  static bool firsttime = true;

  if (firsttime) {
    ct = WAIT;
    firsttime = false;
  }

  switch (ct) {

    case WAIT:
      lightUp(6, 12, 55, 0, 0);
      break;
      
    case FORWARD:
      goStraight(maxSpeed, 0);
      break;

    case BACKWARDS:
      goStraight(-maxSpeed, 0);
      break;

    case TURNING_RIGHT:
      TurningRight(maxSpeed);
      break;

    case TURNING_LEFT:
      TurningLeft(maxSpeed);
      break;
  }
}

void changeSpeed(int newSpeed) {

  if (newSpeed > maxSpeed || newSpeed < minSpeed) {

    Serial.print("Vitesse ");
    Serial.print(newSpeed);
    Serial.println(" hors des limites. (50 - 255)");
  }
  else {

    Serial.println("Nouvelle vitesse: ");
    Serial.println(newSpeed);

    maxSpeed = newSpeed;
  }
}
#pragma endregion

#pragma region - Calibrate ----
  void Calibrating() {

    static bool firstTime = true;
    static bool circleDone = false;

    if (firstTime) {
      firstTime = false;

      gyro.resetData();

      double tempAngle = gyro.getAngleZ();
      zAngleGoal = tempAngle + 360;

      encoderLeft.setMotorPwm(calibratingSpeed);
      encoderRight.setMotorPwm(calibratingSpeed);
    }

    TurningLeft(calibratingSpeed);
    takeInfo();

    const unsigned int waitTime = 300;
    static unsigned long lt = ct;

    if (ct - lt >= waitTime) {

      if (gyro.getAngleZ() > -4 && gyro.getAngleZ() < 4) {
        stop();
        circleDone = true;
        currentState = FOLLOW_LINE;
      }

      stop();
      currentState = FOLLOW_LINE;
    }
  }

  void takeInfo() {

    for (int i = 0; i < nbrCapteurs; i++) {
      capteurs[i].newVal = sees.analogRead(i);

      if (capteurs[i].newVal < capteurs[i].valMin) {
        capteurs[i].valMin = capteurs[i].newVal;
      }

      if (capteurs[i].newVal > capteurs[i].valMax) {
        capteurs[i].valMax = capteurs[i].newVal;
      }

      capteurs[i].seuil = (capteurs[i].valMin + capteurs[i].valMax) / 2;
    }
  }
#pragma endregion

#pragma region - Countdown ----
  void CountingDown() {

  }
#pragma endregion

#pragma region - Going to the line ----
  void GoToLine() {

  }
#pragma endregion

#pragma region - Package seeking ----
  void GettingPackage() {

  }
#pragma endregion

#pragma region - Follow the line ----
void FollowLine() {

  Serial.println("Go time");

  static bool firstTime = true;

  if (firstTime) {


    firstTime = false;
  }

  go();

  bool transToDanger = dist <= thirdyThreshold;

  bool transToPivot = dist <= twentyThreshold;

  if (transToDanger) {
    currentState = LOOK_T;

    firstTime = true;
    return;
  }

  if (transToPivot) {
    currentState = MANUAL;

    firstTime = true;
    return;
  }
}

void go() {

  if (capteurs[2].valNormed < capteurs[2].seuil) {
        // Le capteur du milieu voit la ligne, avancer
        Forward(maxSpeed);
    } else if (capteurs[0].valNormed < capteurs[0].seuil || capteurs[1].valNormed < capteurs[1].seuil) {
        // Les capteurs de gauche voient la ligne, tourner à gauche
        TurningLeft(maxSpeed);
    } else if (capteurs[3].valNormed < capteurs[3].seuil || capteurs[4].valNormed < capteurs[4].seuil) {
        // Les capteurs de droite voient la ligne, tourner à droite
        TurningRight(maxSpeed);
    } else {
        // Aucun capteur ne voit la ligne, arrêter ou chercher la ligne
        stop();
  }
}
#pragma endregion

#pragma region - Looking both ways ----

void Crossing() {

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
    currentState = FOLLOW_LINE;

    firstTime = true;
    return;
  }

  if (transToPivot) {
    currentState = MANUAL;

    firstTime = true;
    return;
  }
}
#pragma endregion

#pragma region - Parking ----
  void Parking() {

  }
#pragma endregion

int getDist(unsigned long ctime) {
  static unsigned long lt;
  const unsigned int waitTime = 175;
  static int lastDistance = 0;
  static int tempDistance = 0;
  static int goodDistance = 0;


  if (ctime - lt >= waitTime) {

    tempDistance = ultrasound.distanceCm();

    if (tempDistance != 0) {

      goodDistance = tempDistance;
      lastDistance = tempDistance;

    }
    else {

      goodDistance = lastDistance;

    }

    lt = ctime;
    return goodDistance;
  }
}

void Forward(int speed) {

  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(-speed);
}

void TurningRight(int speed) {

  encoderLeft.setMotorPwm(speed);
  encoderRight.setMotorPwm(speed);
}

void TurningLeft(int speed) {

  encoderLeft.setMotorPwm(-speed);
  encoderRight.setMotorPwm(-speed);
}

void stop() {
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  encoderLeft.move(0);
  encoderRight.move(0);
}

void lightTheRight(int r = 0, int g = 0, int b = 0, bool changePackColor = false) {
  
  if (changePackColor) {
    // change color if has or not pack
  }
  static int checkpointFirst = 6;
  static int checkpointLast = checkpointFirst + (checkpoint - 1);

  led.setColor(0, 0, 0);   //All off
  lightUpNoResetSend(checkpointFirst, checkpointLast, 50, 0, 50);
  led.show();
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

void lightUpNoResetSend(int startLed, int endLed, int r, int g, int b) {

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