// Milanne Lacerte || DA: 2486739
// Mon robot fait doit donc atteindre une distance donnée.

#include <MeAuriga.h>
#include <Wire.h>
#include <ezBuzzer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

unsigned long ct = 0;   //Temps

float distToStop = 0;
MeGyro gyro(0, 0x69);   // Gyroscope

#pragma region - Global variables ------------

short int cruisingSpeed = 100;
short int pivotSpeed = (cruisingSpeed/1.3);
short int minSpeed = 50;
short int maxSpeed = 255;
String serialMsg = "";
bool debugOn = false;

enum State {WAIT, FORWARD, BACKWARDS, TURNING_RIGHT, TURNING_LEFT, AUTO};
State currentState = WAIT;

#define ALL_LEDS 0
#define LEDNUM  12
#define LED_PIN 44
MeRGBLed led( 0, LEDNUM );

#define BUZZER_PIN 45
ezBuzzer buzzer(BUZZER_PIN);

int melody[] = {
  NOTE_E5
};

int duration[] = {
  10
};

#pragma endregion

#pragma region - Variables for encoder ------------

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
#pragma endregion

#pragma region - Encoder config ------------

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
    const double kp = 10.75;
    //const double ki = 1.0;
    const double kd = 5.0;
    
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

//Loop
void loop() {
  ct = millis();

  switch (currentState) {

    case WAIT:
      Waiting();
      break;
      
    case FORWARD:
      Forward();
      break;

    case BACKWARDS:
      Backwards();
      break;

    case TURNING_RIGHT:
      TurningRight();
      break;

    case TURNING_LEFT:
      TurningLeft();
      break;

    case AUTO:
      autoState();
      break;
  }

  if (debugOn && currentState == AUTO) {
    debugAuto();
  }

  buzzer.loop();
  gyro.update();
  encodersUpdate();
}
//

void encodersUpdate() {
  encoderRight.loop();
  encoderLeft.loop();
}

//Gets the messages and sends the correct input to the robot
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

      if (cmd == "AUTO") {

        distToStop = (secondArg.toFloat() * PULSE_PER_METER);
        
        encoderRight.move(distToStop, cruisingSpeed);
        encoderLeft.move(distToStop, cruisingSpeed);
        goStraight(cruisingSpeed, 1);
        currentState = AUTO;
      }
      else if (cmd == "p") {

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

    goStraight(cruisingSpeed, 1);
    currentState = FORWARD;
  }
  else if (cmd == "B") {
    
    goStraight(-cruisingSpeed, 1);
    currentState = BACKWARDS;
  }
  else if (cmd == "R") {
  
    currentState = TURNING_RIGHT;
  }
  else if (cmd == "L") {

    currentState = TURNING_LEFT;
  }
  else if (cmd == "S") {

    buzzer.stop();
    stop();
    currentState = WAIT;
  }
  else if (cmd == "K") {

    if (buzzer.getState() == BUZZER_IDLE) {
      int length = sizeof(duration) / sizeof(int);
      buzzer.playMelody(melody, duration, length);
    }
  }
  else if (cmd == "d") {

    debugOn = !debugOn;
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

void Waiting() {

  lightUp(6, 12, 55, 0, 0);
}

void Forward() {

  goStraight(cruisingSpeed, 0);
}

void Backwards() {
  
  goStraight(-cruisingSpeed, 0);
  backupSound();
}

void TurningRight() {

  flashingLights(3, 6, 30, 30, 0);

  encoderLeft.setMotorPwm(cruisingSpeed);
  encoderRight.setMotorPwm(cruisingSpeed);
}

void TurningLeft() {

  flashingLights(12, 3, 20, 30, 0);

  encoderLeft.setMotorPwm(-cruisingSpeed);
  encoderRight.setMotorPwm(-cruisingSpeed);
}

void autoState() {

  flashingLights(1, 12, 20, 30, 0);
  goStraight(cruisingSpeed, 0);

  bool transition = encoderLeft.distanceToGo() <= 0;

  if (transition) {

    Serial.println("Ronde automatique complétée.");
    stop();
    currentState = WAIT;
  }
}

void debugAuto() {

  const unsigned int waitTime = 200;
  static unsigned long lt = ct;

  if (ct - lt <= waitTime) return;

  Serial.println(encoderLeft.distanceToGo());
  lt = ct;
}

void backupSound() {
  const unsigned int waitTime = 250;
  static unsigned long lt = ct;
  static bool buzz = false;

  if (ct - lt >= waitTime) {

    buzz = !buzz;

    if (buzz) {

      int length = sizeof(duration) / sizeof(int);
      buzzer.playMelody(melody, duration, length);
    }
    else {

      buzzer.stop();
    }
    lt = ct;
  }
}

void flashingLights(int startLed, int endLed, int r, int g, int b) {

  const unsigned int waitTime = 250;
  static unsigned long lt = ct;
  static bool lightsOn = true;

  if (ct - lt >= waitTime) {

    lightsOn = !lightsOn;

    if (lightsOn) {

      lightUp(startLed, endLed, r, g, b); //starting led, ending led, red, green, blue
    }
    else {

      lightUp(1, 12, 0, 0, 0); //starting led, ending led, red, green, blue
    }
    lt = ct;
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

    cruisingSpeed = newSpeed;
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