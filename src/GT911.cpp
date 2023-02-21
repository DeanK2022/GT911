#include "Arduino.h"
#include "GT911.h"
#include "Wire.h"

// Interrupt handling
volatile uint8_t GT911IRQ = 0;

#ifndef ICACHE_RAM_ATTR
#define ICACHE_RAM_ATTR
#endif

void ICACHE_RAM_ATTR _GT911_irq_handler() {
  noInterrupts();
  GT911IRQ = 1;
  interrupts();
}


// Implementation
GT911::GT911() {

}

void GT911::setHandler(void (*handler)(int8_t, GTPoint*)) {
  touchHandler = handler;
}

bool GT911::begin(uint8_t interruptPin, uint8_t resetPin, uint8_t addr) {
  intPin = interruptPin;
  rstPin = resetPin;
  i2cAddr = addr;

  // Take chip some time to start
  msSleep(300);
  bool result = reset();
  msSleep(200);

  return result;
}


bool GT911::reset() {
  msSleep(1);

  pinOut(intPin);
  pinOut(rstPin);

  pinHold(intPin);
  pinHold(rstPin);

  /* begin select I2C slave addr */

  /* T2: > 10ms */
  msSleep(11);

  /* HIGH: 0x28/0x29 (0x14 7bit), LOW: 0xBA/0xBB (0x5D 7bit) */
  pinSet(intPin, i2cAddr == GT911_I2C_ADDR_28);

  /* T3: > 100us */
  usSleep(110);
  pinIn(rstPin);
  //if (!pinCheck(rstPin, HIGH))
  //  return false;

  /* T4: > 5ms */
  msSleep(6);
  pinHold(intPin);
  /* end select I2C slave addr */

  /* T5: 50ms */
  msSleep(51);
  pinIn(intPin); // INT pin has no pullups so simple set to floating input

  //attachInterrupt(intPin, _GT911_irq_handler, RISING);
  //  detachInterrupt(intPin, _GT911_irq_handler);

  return true;
}

/**
   Read GT911 touchscreen version
   set 4 chars + zero productID to target
*/
uint8_t GT911::productID(char *target) {
  uint8_t error;
  uint8_t buf[4];

  error = read(GT911_REG_ID, buf, 4);
  if (error) {
    return error;
  }

  memcpy(target, buf, 4);
  target[4] = 0;

  return 0;
}

/**
   GT911_i2c_test - I2C test function to check if the device answers.

   @client: the i2c client
*/
uint8_t GT911::test() {
  uint8_t testByte;
  return read(GT911_REG_CONFIG_DATA,  &testByte, 1);
}

uint8_t GT911::calcChecksum(uint8_t* buf, uint8_t len) {
  uint8_t ccsum = 0;
  for (uint8_t i = 0; i < len; i++) {
    ccsum += buf[i];
  }
  //ccsum %= 256;
  ccsum = (~ccsum) + 1;
  return ccsum;
}

uint8_t GT911::readChecksum() {
  uint16_t aStart = GT_REG_CFG;
  uint16_t aStop = 0x80FE;
  uint8_t len = aStop - aStart + 1;
  uint8_t buf[len];

  read(aStart, buf, len);
  return calcChecksum(buf, len);
}

uint8_t GT911::fwResolution(uint16_t maxX, uint16_t maxY) {
  uint8_t len = 0x8100 - GT_REG_CFG + 1;
  uint8_t cfg[len];
  read(GT_REG_CFG, cfg, len);

  cfg[1] = (maxX & 0xff);
  cfg[2] = (maxX >> 8);
  cfg[3] = (maxY & 0xff);
  cfg[4] = (maxY >> 8);
  cfg[len - 2] = calcChecksum(cfg, len - 2);
  cfg[len - 1] = 1;

  return write(GT_REG_CFG, cfg, len);
}

GTConfig* GT911::readConfig() {
  read(GT_REG_CFG, (uint8_t *) &config, sizeof(config));
  return &config;
}

GTInfo* GT911::readInfo() {
  read(GT_REG_DATA, (uint8_t *) &info, sizeof(config));
  return &info;
}

//void GT911::touchHandler() {
//  attachInterrupt(intPin, _GT911_irq_handler, RISING);
//}

void GT911::onIRQ() {
  int16_t readResult = readInput((uint8_t *)points);
  if (readResult < 0) {
//    Serial.print(millis());
//    Serial.printf(" Error: %d\n", readResult);
    return;
  }

  contacts = readResult;
  if (contacts > 0) {
    if (touchHandler) {
      touchHandler(contacts, points);
    }
/*
    Serial.print(millis());
    Serial.print(" Contacts: ");
    Serial.println(contacts);

    for (uint8_t i = 0; i < contacts; i++) {
      Serial.print("C ");
      Serial.print(i);
      Serial.print(": #");
      Serial.print(points[i].trackId);
      Serial.print(" ");
      Serial.print(points[i].x);
      Serial.print(", ");
      Serial.print(points[i].y);
      Serial.print(" s.");
      Serial.print(points[i].area);
      Serial.println();
    }
*/
  }

  write(GT911_READ_COORD_ADDR, 0);
}

void GT911::loop() {
  noInterrupts();
  uint8_t irq = GT911IRQ;
  GT911IRQ = 0;
  interrupts();

  if (irq) {
    onIRQ();
  }
}

#define GT911_EAGAIN 100 // Try again error

int16_t GT911::readInput(uint8_t *data) {
  int touch_num;
  int error;

  uint8_t regState[1];

  error = read(GT911_READ_COORD_ADDR, regState, 1);
  //Serial.printf("regState:%x\n",regState[0]);

  if (error) {
    Serial.printf("read error:%d\n",error);
    return -error;
  }

  if (!(regState[0] & 0x80))
    return -GT911_EAGAIN;

  touch_num = regState[0] & 0x0f;

  if (touch_num > 0) {

     error = read(GT911_READ_COORD_ADDR, data, GT911_CONTACT_SIZE * (touch_num));

    if (error)
      return -error;
  }
  contacts = touch_num;
  return touch_num;
}

GTPoint* GT911::readPoints()
{
    int16_t readResult = readInput((uint8_t*)points);
  
    write(GT911_READ_COORD_ADDR, 0);
  
    if (readResult < 0) {
        //    Serial.print(millis());
        //Serial.printf(" Error: %d\n", readResult);
        return &points[0];
    }
 
    return &points[0];

}
//----- Utils -----
void GT911::i2cStart(uint16_t reg) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
}

void GT911::i2cRestart() {
  Wire.endTransmission(false);
  Wire.beginTransmission(i2cAddr);
}

uint8_t GT911::i2cStop() {
  return Wire.endTransmission(true);
}

uint8_t GT911::write(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t error;
  uint16_t startPos = 0;

  while (startPos < len) {
    i2cStart(reg + startPos);
    startPos += Wire.write(buf + startPos, len - startPos);
    error = Wire.endTransmission();
    if (error)
      return error;
  }
  return 0;
}

uint8_t GT911::write(uint16_t reg, uint8_t buf) {
  i2cStart(reg);
  Wire.write(buf);
  return Wire.endTransmission();
}

uint8_t GT911::read(uint16_t reg, uint8_t *buf, size_t len) {
  uint8_t res;

  i2cStart(reg);

  res = Wire.endTransmission(false);
  if (res != GT911_OK) {
    return res;
  }

  uint16_t pos = 0, prevPos = 0;
  uint8_t maxErrs = 3;

  while (pos < len) {
    Wire.requestFrom(i2cAddr, (len - pos));

    prevPos = pos;
    while (Wire.available()) {
      buf[pos] = Wire.read();
      pos++;
    }

    if (prevPos == pos)
      maxErrs--;

    if (maxErrs <= 0) {
      break;
    }
    delay(0);
  }
  return 0;
}

void GT911::pinOut(uint8_t pin) {
  pinMode(pin, OUTPUT);
}

void GT911::pinIn(uint8_t pin) {
  pinMode(pin, INPUT);
}

void GT911::pinSet(uint8_t pin, uint8_t level) {
  digitalWrite(pin, level);
}

void GT911::pinHold(uint8_t pin) {
  pinSet(pin, LOW);
}

bool GT911::pinCheck(uint8_t pin, uint8_t level) {
  return digitalRead(pin) == level;
}

void GT911::msSleep(uint16_t milliseconds) {
  delay(milliseconds);
}

void GT911::usSleep(uint16_t microseconds) {
  delayMicroseconds(microseconds);
}
TS_Point GT911::readTS_Points()
{
    int16_t readResult = readInput((uint8_t*)points);
    write(GT911_READ_COORD_ADDR, 0);

    if (readResult < 0) {
        //    Serial.print(millis());
        //    Serial.printf(" Error: %d\n", readResult);
        return TS_Point(0,0,0,0);
    }
    uint8_t* point = (uint8_t*)(&points[0]);
    return TS_Point(
        (((uint16_t)point[2]) << 8) + point[1],  // x
        (((uint16_t)point[4]) << 8) + point[3],  // y
        (((uint16_t)point[6]) << 8) + point[5],  // area
        point[0]); // id
}

TS_Point GT911::getPoint(uint8_t n) {
  loop();
  if ((contacts == 0) || (n >= contacts)) {
    return TS_Point(0, 0, 0, 0);
  } else {
    uint8_t* point = (uint8_t*)(&points[n]);
    /*  point structure:
        point[0] - id
        point[1] - x low
        point[2] - x high
        point[3] - y low
        point[4] - y high
        point[5] - area low
        point[6] - area high
        point[7] - reserved
    */
    return TS_Point(
      (((uint16_t)point[2])<<8) + point[1],  // x
      (((uint16_t)point[4])<<8) + point[3],  // y
      (((uint16_t)point[6])<<8) + point[5],  // area
      point[0]); // id
    return TS_Point(points[n].x, points[n].y, points[n].area, points[n].trackId);
  }
}
GTPoint* GT911::getPoints() {
    return &points[0];

}

uint8_t GT911::touched() {
  loop();
  return contacts;
}

uint8_t GT911::touchcount() {
    
    int error;

    uint8_t regState[1];

    error = read(GT911_READ_COORD_ADDR, regState, 1);
    if (error)
    {
        Serial.printf("Error reading coords %d", error);
        return false;
    }
    //uint8_t pointInfo = regState[0];
    //uint8_t bufferStatus = pointInfo >> 7 & 1;
    //uint8_t proximityValid = pointInfo >> 5 & 1;
    //uint8_t haveKey = pointInfo >> 4 & 1;
    //uint8_t isLargeDetect = pointInfo >> 6 & 1;

    //if ((regState[0] & 0x0f) > 0)
    //    Serial.printf("pointInfo %d, bufferStatus %d, proximityValid %d, haveKey %d, isLargeDetect %d\n", pointInfo, bufferStatus, proximityValid, haveKey, isLargeDetect);

    contacts = regState[0] & 0x0f;
    return (regState[0] & 0x0f );

}



void GT911::dumpConfig(GTConfig* cfg)
{   
    //little endian

    Serial.printf("configVersion %d\n",  cfg->configVersion);
    Serial.printf("xResolution %d\n", cfg->xResolution);
    Serial.printf("yResolution %d\n", cfg->yResolution);
    Serial.printf("touchNumber %d\n", cfg->touchNumber);
    Serial.printf("moduleSwitch1 %d\n", cfg->moduleSwitch1);
    Serial.printf("moduleSwitch2 %d\n", cfg->moduleSwitch2);
    Serial.printf("shakeCount %d\n", cfg->shakeCount);
    Serial.printf("filter %d\n", cfg->filter);
    Serial.printf("largeTouch %d\n", cfg->largeTouch);
    Serial.printf("noiseReduction %d\n", cfg->noiseReduction);
    Serial.printf("screenLevel touch %d leave %d\n", cfg->screenLevel.touch, cfg->screenLevel.leave);
    Serial.printf("lowPowerControl %d\n", cfg->lowPowerControl);
    Serial.printf("refreshRate %d\n", cfg->refreshRate);
    Serial.printf("xThreshold %d\n", cfg->xThreshold);
    Serial.printf("yThreshold %d\n", cfg->yThreshold);
    Serial.printf("xSpeedLimit %d\n", cfg->xSpeedLimit);
    Serial.printf("ySpeedLimit %d\n", cfg->ySpeedLimit);
    Serial.printf("vSpace %d\n", cfg->vSpace);
    Serial.printf("hSpace %d\n", cfg->hSpace);
    Serial.printf("stretchRate %d\n", cfg->stretchRate);
    Serial.printf("stretchR0 %d\n", cfg->stretchR0);
    Serial.printf("stretchR1 %d\n", cfg->stretchR1);
    Serial.printf("stretchR2 %d\n", cfg->stretchR2);
    Serial.printf("stretchRM %d\n", cfg->stretchRM);
    Serial.printf("drvGroupANum %d\n", cfg->drvGroupANum);
    Serial.printf("drvGroupBNum %d\n", cfg->drvGroupBNum);
    Serial.printf("sensorNum %d\n", cfg->sensorNum);
    Serial.printf("freqAFactor %d\n", cfg->freqAFactor);
    Serial.printf("freqBFactor %d\n", cfg->freqBFactor);
    Serial.printf("pannelBitFreq %d\n", cfg->pannelBitFreq);
    Serial.printf("pannelSensorTime %d\n", cfg->pannelSensorTime);
    Serial.printf("pannelTxGain %d\n", cfg->pannelTxGain);
    Serial.printf("pannelRxGain %d\n", cfg->pannelRxGain);
    Serial.printf("pannelDumpShift %d\n", cfg->pannelDumpShift);
    Serial.printf("drvFrameControl %d\n", cfg->drvFrameControl);
    Serial.printf("stylusConfig txGain %d rxGain %d dumpShift %d level.touch %d level.leave %d control %d\n", cfg->stylusConfig.txGain, cfg->stylusConfig.rxGain, cfg->stylusConfig.dumpShift, cfg->stylusConfig.level.touch, cfg->stylusConfig.level.leave, cfg->stylusConfig.control);
    Serial.printf("freqHoppingStart %d\n", cfg->freqHoppingStart);
    Serial.printf("freqHoppingEnd %d\n", cfg->freqHoppingEnd);
    Serial.printf("noiseDetectTims %d\n", cfg->noiseDetectTims);
    Serial.printf("hoppingFlag %d\n", cfg->hoppingFlag);
    Serial.printf("hoppingThreshold %d\n", cfg->hoppingThreshold);
    Serial.printf("noiseThreshold %d\n", cfg->noiseThreshold);

    for (int idx = 0; idx < 5; idx++)
        Serial.printf("hoppingSegments[%d] hoppingBitFreq %d hoppingFactor%d\n", idx, cfg->hoppingSegments[idx].hoppingBitFreq, cfg->hoppingSegments[idx].hoppingFactor);

    Serial.printf("keys pos1 %d pos2 %d pos3 %d pos4 %d area %d level.touch %d cfg->keys.level.leave %d sens12 %d cfg->keys.sens34 %d cfg->keys.restrain %d \n", cfg->keys.pos1, cfg->keys.pos2, cfg->keys.pos3, cfg->keys.pos4,
    cfg->keys.area, cfg->keys.level.touch, cfg->keys.level.leave, cfg->keys.sens12, cfg->keys.sens34, cfg->keys.restrain);

    
}   

TS_Point::TS_Point(void) {
  x = y = z = id = 0;
}

TS_Point::TS_Point(int16_t _x, int16_t _y, int16_t _z, int16_t _id) {
  x = _x;
  y = _y;
  z = _z;
  id = _id;
}

bool TS_Point::operator==(TS_Point p1) {
  return  ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

bool TS_Point::operator!=(TS_Point p1) {
  return  ((p1.x != x) || (p1.y != y) || (p1.z != z));
}




