#include <Wire.h>

#define I2C_ADDR 0x52
#define MULTIPLEXER_ADDR 0x70
#define NUM_OF_COLOR_SENSORS 2
#define OUTPUT_PIN_START 2 //six pins after and including this will be used to output color sensor state

#define PROXIMITY_LIMIT 40 //the output from proximity goes up as object gets closer, this is the amount it needs to be at to trigger the output pin

uint8_t readBuff[16];
uint16_t proximity;
uint16_t ir=0;
uint16_t red=0;
uint16_t green=0;
uint16_t blue=0;
float maxV = 0;

// these values calibrate the color sensors so they read colors correctly. calibrate by bringing sensor to white paper and then setting these to the raw values given by color sensor
float whiteValues[NUM_OF_COLOR_SENSORS][3] = {
                                              {7496, 13430, 6685},
                                              {6176, 11008, 5371}
                                             };
                                             
enum registry {
  sensorMode = 0x00,
  lightSensorMeasurementRate = 0x04,
  proximitySensorMeasurementRate = 0x03,
  infraredData = 0x0A,
  proximityData = 0x08
};

enum sensorModes {
  color = 0x04,
  light = 0x02,
  prox = 0x01
};

enum proximitySensorResolution {
  proxRes8bit = 0x00,
  proxRes10bit = 0x10,
  proxRes11bit = 0x18
};

enum proximitySensorMeasurementRates {
  proxRate25ms = 0x03,
  proxRate50ms = 0x04
};

enum colorSensorResolution {
  colorRes20bit = 0x00,
  colorRes16bit = 0x40
};

enum colorSensorMeasurementRate {
  colorRate25ms = 0,
  colorRate50ms = 1
};

void setup() {
  Wire.begin();
  Serial.begin(9600);
  i2cWrite(registry::sensorMode, sensorModes::color | sensorModes::light | sensorModes::prox);  //enable light sensor, activate rgb mode and activate proximity sensor
  i2cWrite(registry::lightSensorMeasurementRate, colorSensorResolution::colorRes16bit | colorSensorMeasurementRate::colorRate25ms); //set to 16 bit resolution for 25ms response time and set measurement rate to 25ms

  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  for (uint8_t i = 0; i < 6; i++) {
    pinMode(OUTPUT_PIN_START+i, OUTPUT);
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_OF_COLOR_SENSORS; i++) {
    i2cMultiplexerWrite(0x70, byte(i+1));
    i2cRead(registry::proximityData, readBuff, 16);

    proximity = (readBuff[1]<<8) | readBuff[0];
    ir = (readBuff[4]<<8) | readBuff[3];
    green = (readBuff[7]<<8) | readBuff[6];
    blue = (readBuff[10]<<8) | readBuff[9];
    red = (readBuff[13]<<8) | readBuff[12];

    red*=whiteValues[i][1] / whiteValues[i][0];
    green*=whiteValues[i][1] / whiteValues[i][1];
    blue*=whiteValues[i][1] / whiteValues[i][2];

    //Normalize the readings to brightest channel then apply log scale to better discern the colors.
    maxV=max(blue,max(red,green));
    red=255*pow(red/maxV,5);
    green=255*pow(green/maxV,5);
    blue=255*pow(blue/maxV,5);

    if (red > 254 && green < 10 && blue < 10) {
      digitalWrite(OUTPUT_PIN_START+(i*3), HIGH);
      digitalWrite(OUTPUT_PIN_START+(i*3)+1, LOW);
    } else if (blue > 254 && red < 10 && green < 10) {
      digitalWrite(OUTPUT_PIN_START+(i*3), LOW);
      digitalWrite(OUTPUT_PIN_START+(i*3)+1, HIGH);
    } else {
      digitalWrite(OUTPUT_PIN_START+(i*3), LOW);
      digitalWrite(OUTPUT_PIN_START+(i*3)+1, LOW);
    }

    if (proximity >= PROXIMITY_LIMIT) {
      digitalWrite(OUTPUT_PIN_START+(i*3)+2, HIGH);
    } else {
      digitalWrite(OUTPUT_PIN_START+(i*3)+2, LOW);
    }
  }
  delay(25);
}


void i2cWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}
void i2cMultiplexerWrite(uint8_t reg, uint8_t val){
    Wire.beginTransmission(MULTIPLEXER_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}
void i2cRead(uint8_t reg,uint8_t *val,uint16_t len){
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, len);
    for(uint8_t i=0;i<len;i++){
      val[i]=Wire.read();
    }
}
