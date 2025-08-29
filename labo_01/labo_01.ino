#include <HCSR04.h>


HCSR04 ultrasound(2, 3);
float dist = 0;


void setup() {
  Serial.begin(115200);
  dist = ultrasound.dist();

}

void loop() {
  getDist();

}

float getDist() {
  
  return
}