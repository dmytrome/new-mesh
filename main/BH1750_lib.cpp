
#include <BH1750.h>
#include <Wire.h>

#define SDA_PIN 5
#define SCL_PIN 4

BH1750 lightMeter(0x23);

void bh1750_init(void)
{
   Wire.begin(SDA_PIN, SCL_PIN);

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initialising BH1750"));
  }
}

float bh1750_get_lx(void)
{
  if (lightMeter.measurementReady()) {
    return  lightMeter.readLightLevel();
  }
  else
  {
    return -1;
  }
}
