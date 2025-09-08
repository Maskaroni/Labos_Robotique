
// Ultrasons
#define isRobot 0

#if isRobot
  #include <MeAuriga.h>
  MeUltrasonicSensor ultrasound(PORT_10);
#else
  #include <HCSR04.h>
  HCSR04 ultrasound(2, 3);
#endif

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

unsigned long ct = 0;

//Écran Oled
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET  -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 screen(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//    Sun
int sunMovement = 0;   //48 = hautD -- 0 = basD
#define sunH  16
#define sunW  16
static const unsigned char PROGMEM sun[] =
{
  0x04, 0x40, 0x22, 0x44, 0x12, 0x49, 0x08, 0x92, 0x80, 0x00, 0x63, 0xc6, 0x16, 0xe8, 0x05, 0xe4, 0x17, 0xe0, 0x67, 0xed, 0x03, 0xc2, 0x18, 0x10, 0x22, 0x48, 0x4a, 0x44, 0x09, 0x20, 0x00, 0x20
  // 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x03, 0xf0, 0x30, 0xc3, 0xfc, 0x30, 0x10, 0x20, 0x00, 0x60, 0x18, 0x7f, 0x00, 0xc0, 0x0c, 0x7f, 0xc3, 0x00, 0x06, 0x20, 0x7e, 0x00, 0x03, 0xc3, 0x23, 0x00, 
  // 0x00, 0xe3, 0x33, 0x00, 0x01, 0xbe, 0x7e, 0x00, 0x1f, 0x1f, 0xf8, 0x00, 0x06, 0x78, 0x08, 0x00, 0x07, 0xf0, 0x08, 0x00, 0x00, 0xc0, 0x78, 0x00, 0x01, 0x98, 0x00, 0x00, 0x01, 0xcc, 0x18, 0x00,
  // 0x00, 0x7c, 0x08, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x20, 0x08, 0x00, 0x00, 0x22, 0x38, 0x00, 0x00, 0x23, 0xf8, 0x00, 0x00, 0x21, 0x98, 0x00, 0x00, 0x21, 0x1c, 0x00, 0x00, 0x21, 0x14, 0x00, 0x00, 0x27, 0x1c, 0x00, 0x00, 0x2d, 0x0c, 0x00, 0x00, 0x39, 0x0c, 0x00, 0x00, 0x21, 0x0c, 0x00, 0x00, 0x21, 0x66, 0x00, 0x00, 0x61, 0x76, 0x00, 0x00, 0x41, 0xfd, 0x00
};

// Variables globales
int dist = 0;
int sunUpDist = 10;
int sunDownDist = 50;
int sunDown = 0;
int sunUp = 48;
// -


void setup() {
  Serial.begin(115200);
#if isRobot
  dist = ultrasound.distanceCm();
#else
  dist = ultrasound.dist();
#endif
  displayIni();
}

void loop() {
  ct = millis();
  getDist(ct);
  screenDisp(ct);
}

void displayIni() {
  if(!screen.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec d'allocation SSD1306"));
    for(;;);
  }
  screen.display();
  delay(2000);  //Delay dans le setup est good, tu as dis à la dernière session!

  screen.clearDisplay();
}

void getDist(unsigned long ct) {
  unsigned static long lt;
  const unsigned int waitTime = 100;
  static int lastDistance = 0;
  static int tempDistance;

  if (ct - lt >= waitTime) {
    
#if isRobot
    tempDistance = ultrasound.distanceCm();
#else
    tempDistance = ultrasound.dist();
#endif

    if (tempDistance != 0) {
      dist = tempDistance;
      lastDistance = dist;
    }
    else {
      dist = lastDistance;
    }
    lt = ct;
    writeDist();
  }
}

void writeDist() {
  Serial.print("Distance : ");
  Serial.print(dist);
  Serial.println(" cm");
}

void screenDisp(unsigned long ct) {
  unsigned static long lt;
  const unsigned int waitTime = 100;
  static int lastDistance = 0;
  static int tempDistance;

  getNewSunMovement();

  if (ct - lt >= waitTime) {
    refreshScreen();
  }
}

void getNewSunMovement() {
  if (dist >= sunDownDist) {
    sunMovement = sunUp;
  }
  else if (dist <= sunUpDist) {
    sunMovement = sunDown;
  }
  else {
    sunMovement = map(dist, sunUpDist, sunDownDist, sunDown, sunUp);
  }
}

void refreshScreen() {
  screen.clearDisplay();

  screen.drawBitmap(
  (screen.width()  - sunW ),
  (screen.height() - sunH) - sunMovement,
  sun, sunW, sunH, 1);

  screen.fillTriangle(0, 35, 10, 20, 20, 35, SSD1306_WHITE);  //Roof
  screen.fillRect(3, 30, 15, 16, SSD1306_WHITE);  //Walls
  screen.fillRect(5, 38, 5, 8, SSD1306_INVERSE);  //Door
  screen.fillRect(12, 36, 4, 4, SSD1306_INVERSE);  //Window

  screen.setTextSize(1); // Draw 2X-scale text
  screen.setTextColor(SSD1306_WHITE);
  screen.setCursor(0, 0);
  screen.println(F("Lacerte"));

  screen.display();
}