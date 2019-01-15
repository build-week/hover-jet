//
// Load-Cell Sketch
//
#include "HX711.h"

#define DEFAULT_GAIN_VALUE 128

HX711 X0(22, 23, DEFAULT_GAIN_VALUE);
HX711 Y0(25, 24, DEFAULT_GAIN_VALUE);
HX711 Y1(26, 27, DEFAULT_GAIN_VALUE);
HX711 Z0(28, 29, DEFAULT_GAIN_VALUE);
HX711 X1(31, 30, DEFAULT_GAIN_VALUE);
HX711 Z1(32, 33, DEFAULT_GAIN_VALUE);

void setup() {
  Serial.begin();

  //scale_1.set_scale(CALIBRATION_20KG_SCALE_1);
  //scale_1.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
}

void loop() {
  const float X0_read = X0.get_units();
  const float Y0_read = Y0.get_units();
  const float Y1_read = Y1.get_units();
  const float Z0_read = Z0.get_units();
  const float X1_read = X1.get_units();
  const float Z1_read = Z1.get_units();

  byte * X0_byte_rep = (byte *) &X0_read;
  byte * Y0_byte_rep = (byte *) &Y0_read;
  byte * Y1_byte_rep = (byte *) &Y1_read;
  byte * Z0_byte_rep = (byte *) &Z0_read;
  byte * X1_byte_rep = (byte *) &X1_read;
  byte * Z1_byte_rep = (byte *) &Z1_read;
  

  Serial.print("SENSOR0,"); Serial.write(X0_byte_rep, 4); Serial.println(";");
  Serial.print("SENSOR1,"); Serial.write(Y0_byte_rep, 4); Serial.println(";");
  Serial.print("SENSOR2,"); Serial.write(Y1_byte_rep, 4); Serial.println(";");
  Serial.print("SENSOR3,"); Serial.write(Z0_byte_rep, 4); Serial.println(";");
  Serial.print("SENSOR4,"); Serial.write(X1_byte_rep, 4); Serial.println(";");
  Serial.print("SENSOR5,"); Serial.write(Z1_byte_rep, 4); Serial.println(";");
  
  
  //Serial.print("SENSOR0,"); Serial.print(Y1_read); Serial.println(";");

}
