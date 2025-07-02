#include "NPK_lib.hpp"
#include <Arduino.h>
#include "HardwareSerial.h"
#define RXD0 36  // OK!
#define TXD0 35  // OK!

void NPK_init(void)
{
  Serial1.begin(4800, SERIAL_8N1, RXD0, TXD0);
  delay(1000);
}
void NPK_get(npk_data_t* npk_data)
{
   byte Soil_sensor_request[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08 };
  Serial1.write(Soil_sensor_request, sizeof(Soil_sensor_request));
  Serial1.flush();
  byte Soil_sensor_buf[24] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  Serial1.readBytes(Soil_sensor_buf, 24);

  // Parse sensor parameters
  npk_data->humidity = word(Soil_sensor_buf[3], Soil_sensor_buf[4]);
  npk_data->humidity /= 10.0;

  npk_data->temperature = word(Soil_sensor_buf[5], Soil_sensor_buf[6]);
  npk_data->temperature /= 10.0;

  npk_data->conductivity = word(Soil_sensor_buf[7], Soil_sensor_buf[8]);

  npk_data->pH = word(Soil_sensor_buf[9], Soil_sensor_buf[10]);
  npk_data->pH /= 10.0;

  npk_data->Nitrogen = word(Soil_sensor_buf[11], Soil_sensor_buf[12]);
  npk_data->Phosphorus = word(Soil_sensor_buf[13], Soil_sensor_buf[14]);
  npk_data->Potassium = word(Soil_sensor_buf[15], Soil_sensor_buf[16]);
  npk_data->Salinity = word(Soil_sensor_buf[17], Soil_sensor_buf[18]);
  npk_data->TDS = word(Soil_sensor_buf[19], Soil_sensor_buf[20]);
}