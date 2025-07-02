#include <AM2302-Sensor.h>

constexpr unsigned int SENSOR_PIN {14U};

AM2302::AM2302_Sensor am2302{SENSOR_PIN};

void am2302_init(void)
{
   if (am2302.begin()) {
      // this delay is needed to receive valid data,
      // when the loop directly read again
      delay(3000);
   }   
   else {
      while (true) {
      Serial.println("Error: sensor check. => Please check sensor connection!");
      delay(10000);
      }
   }
}

void am2302_get_data(float* T, float* H)
{
  auto status = am2302.read();
  *T = am2302.get_Temperature();
  *H = am2302.get_Humidity();
}
