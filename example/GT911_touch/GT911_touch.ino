#include "Arduino.h"
#include <Wire.h>
#include "GT911.h"

#define INT_PIN 25
#define RST_PIN 26


GT911 touch = GT911();

// note: GTPoint uint16_t are always little endian
void handleTouch(int8_t contacts, GTPoint *points) {
  Serial.printf("Contacts: %d Time: %lu\n", contacts,millis());
  for (uint8_t i = 0; i < contacts; i++) {
    Serial.printf("C%d: #%d %d,%d s:%d\n", i, points[i].trackId, points[i].x, points[i].y, points[i].area);
    yield();
  }
}

void touchStart() {
 if (touch.begin(INT_PIN, RST_PIN,GT911_I2C_ADDR_BA)!=true) {
    Serial.println("! Module reset failed");
  } else {
    Serial.println("Module reset OK");
  }

  Serial.print("Check ACK on addr request on 0x");
  Serial.print(touch.i2cAddr, HEX);

  Wire.beginTransmission(touch.i2cAddr);
  int error = Wire.endTransmission();
  if (error == 0) {
    Serial.println(": SUCCESS");
  } else {
    Serial.print(": ERROR #");
    Serial.println(error);
  }
}

void setup() {
  Serial.begin(115200);
  
  Wire.setClock(400000);
  Wire.begin();
  delay(300);

  //touch.setHandler(handleTouch);
  touchStart();
  GTInfo * info = touch.readInfo();
  Serial.printf("Info:\n  productId %c%c%c%c\n  fwId %d\n  vendorId %d\n  xResolution %d\n  yResolution %d lastch %d\n", info->productId[0],info->productId[1],
  info->productId[2], info->productId[3], info->fwId,                info->vendorId, info->xResolution, info->yResolution, info->productId[3]);

  GTConfig *cfg = touch.readConfig();
  Serial.println("Config");
  touch.dumpConfig(cfg);

  uint8_t chkCalc = touch.calcChecksum((uint8_t *) cfg,  0x8100 - GT_REG_CFG  -1);
  uint8_t chkRead = touch.readChecksum();
  Serial.printf("Checksum read %d calc %d\n", chkRead, chkCalc);
}

void loop() {
  uint8_t touches = touch.touchcount();
   
  if (touches >= 0)// || touches == 0)
  {
    GTPoint *myPoints = touch.readPoints();
    for (uint8_t idx = 0; idx < touches; idx++)
    {
      Serial.printf("Touches %d, x %d, y %d area %d, trackID %d\n", touches, myPoints->x, myPoints->y, myPoints->area, myPoints->trackId);
      myPoints++;
    }
  }
  delay(1);
}
