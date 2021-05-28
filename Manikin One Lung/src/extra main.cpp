#include <Arduino.h>

//delay in reading
int dly = 100;

//define pins on the ESP
#define pressurePin_L A0
#define airflowRawPin_L A1
#define airtempPin_L A2

#define pressurePin_R A3
#define airflowRawPin_R A4
#define airtempPin_R A5

//define variables
float pressure_ADunits;
float pressure_Volts;
float normalPressure_L;
float normalPressure_R;
float pressure_H2Ocm;

// -----------------AIRFLOW CODE Paul Badger----------------------

// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it.

const float zeroWindAdjustment = .3; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits; //temp termistor value from wind sensor
float RV_Wind_ADunits; //RV output from wind sensor
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;
float WindSpeed_ms;

//add radius in mm --> get ml/s
float readAirflow(float radius, uint8_t pin_name_airflow, uint8_t pin_name_temp)
{
  // if (millis() - lastMillis > dly)
  //{ // read every 200 ms - printing slows this down further

  TMP_Therm_ADunits = analogRead(pin_name_temp);
  RV_Wind_ADunits = analogRead(pin_name_airflow);
  RV_Wind_Volts = (RV_Wind_ADunits * 0.0048828125);

  // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
  // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
  TempCtimes100 = (0.005 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;

  zeroWind_ADunits = -0.0006 * ((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172; //  13.0C  553  482.39

  zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;

  WindSpeed_MPH = pow(((RV_Wind_Volts - zeroWind_volts) / .2300), 2.7265);
  WindSpeed_ms = WindSpeed_MPH * 0.44704;

  // Serial.print("  TMP volts ");
  // Serial.print(TMP_Therm_ADunits * 0.0048828125);

  // Serial.print(" RV volts: ");
  // Serial.print((float)RV_Wind_Volts);

  // Serial.print(" RV read: ");
  // Serial.print(RV_Wind_ADunits);

  // Serial.print("\t  TempC*100: ");
  // Serial.print(TempCtimes100);

  // Serial.print("   ZeroWind volts: ");
  // Serial.print(zeroWind_volts);

  // Serial.print("   WindSpeed MPH ");
  // Serial.println((float)WindSpeed_MPH);

  // Serial.print("   WindSpeed m/s: ");
  // Serial.print((float)WindSpeed_ms);

  float pi = 3.14150;
  float airVolume = WindSpeed_ms * radius * radius * pi;

  // Serial.print("   Flow in ml/s: ");
  // Serial.print(airVolume);

  return airVolume;

  // lastMillis = millis();
  // }
}

// ------------------FUNCTIONS------------------------------------

//print thresholds for danger
void dangerPressure(float danger)
{
  Serial.print(danger);
  Serial.print(',');
}

void dangerAirflow(float danger)
{
  Serial.print(danger);
}

//float map by Eric
float float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Test function to test serial communication
void sendTestVariables()
{
  float breath = (exp(sin(millis() / 2000.0 * PI)) - 0.36787944) * 108.0;
  Serial.print(breath);
  Serial.print(',');
  Serial.print(breath * 3);
  Serial.print(',');
  Serial.print(200);
  Serial.print(',');
  Serial.println(385);
}

//calibrate the pressure sensor in the first 10 seconds to get the "normal" atmospheric pressure
float calibratePressure(uint8_t pin_name)
{
  float normalPressureSum_Volts = 0;
  float normalPressure_Volts;
  float normalPressure_H2Ocm;
  for (int i = 0; i < 50; i++)
  {
    pressure_ADunits = analogRead(pin_name);
    pressure_Volts = (pressure_ADunits * 0.0048828125);
    normalPressureSum_Volts += pressure_Volts;
    delay(100);
  }
  normalPressure_Volts = normalPressureSum_Volts / 50;
  Serial.print("Normal pressure VOLTAGE is: ");
  Serial.println(normalPressure_Volts);

  normalPressure_H2Ocm = float_map(normalPressure_Volts, 0.17, 4.82, -67.97, 67.97); //map pressure in volts to cmH2O
  Serial.print("Normal pressure cmH2O is: ");
  Serial.println(normalPressure_H2Ocm);

  return normalPressure_H2Ocm;
}

//read the pressure in mV and map it to cm/H2O then send it over the serial
float readPressure(uint8_t pin_name, float normalPressure)
{
  pressure_ADunits = analogRead(pin_name);
  pressure_Volts = (pressure_ADunits * 0.0048828125);
  pressure_H2Ocm = float_map(pressure_Volts, 0.17, 4.82, -67.97, 67.97); //map pressure in volts to cmH2O

  // Serial.print("   Pressure Volts: ");
  // Serial.print(pressure_Volts, 2);

  // Serial.print("   Pressure (cmH2O): ");
  //Serial.print(pressure_H2Ocm, 2);

  // Serial.print("   Pressure difference(cmH2O): ");
  //Serial.print((pressure_H2Ocm - normalPressure), 2);

  float x = pressure_H2Ocm - normalPressure;
  return x;
}

void averageAndDirection(float &pressure_L, float *prev_pressure_L, int &direction_L)
{
  //rolling average L
  if (pressure_L < 1.5)
  {
    pressure_L = (pressure_L + prev_pressure_L[0] + prev_pressure_L[1] + prev_pressure_L[2]) / 4;
  }

  //calculate airflow direction L
  if ((pressure_L + 0.05 < prev_pressure_L[2]) && (prev_pressure_L[2] < prev_pressure_L[1]) && (prev_pressure_L[1] < prev_pressure_L[0]))
  {
    direction_L = -1;
  }
  else if ((pressure_L > prev_pressure_L[2] + 0.05) && (prev_pressure_L[2] > prev_pressure_L[1]) && (prev_pressure_L[1] > prev_pressure_L[0]))
  {
    direction_L = 1;
  }

  //update previous values L
  for (int i = 0; i < 2; i++)
  {
    prev_pressure_L[i] = prev_pressure_L[i + 1];
  }
  prev_pressure_L[2] = pressure_L;
}

//calculate tidal volume by adding airflow*time to a variable
void tidalVolume(float airflow, float &volume, float direction, float delay)
{
  if ((airflow > 0.5) || (airflow < -0.5))
    volume = volume + (((airflow * delay) / 1000) * direction);

  if (volume < 0)
    volume = 0;
}

// ---------------------------------------------------------------- //
// ------------------------ MAIN CODE ----------------------------- //
// ---------------------------------------------------------------- //

//define loop variables
float len = 2.5;
float pressure_L;
float airflow_L;
float pressure_R;
float airflow_R;
int direction_R = 1;
int direction_L = 1;
float prev_pressure_R[3];
float prev_pressure_L[3];
float volume_L = 0;
float volume_R = 0;

void setup()
{
  //Start Serial coms
  Serial.begin(9600);

  //connectToOOCSI();

  //pin modes
  // pinMode(pressurePin_L, INPUT);
  pinMode(airflowRawPin_L, INPUT);
  pinMode(airtempPin_L, INPUT);
  pinMode(pressurePin_R, INPUT);
  pinMode(airflowRawPin_R, INPUT);
  pinMode(airtempPin_R, INPUT);

  Serial.println("Respiratory Monitoring Extra Manikin V1");

  //calibrate pressure sensor for 10 seconds (this allows for the flow sensor to calibrate as well)
  // Serial.println("Calibrating Left Lung Pressure");
  // normalPressure_L = calibratePressure(pressurePin_L);
  Serial.println("Calibrating Lung Pressure");
  normalPressure_R = calibratePressure(pressurePin_R);

  for (int i = 0; i < 3; i++)
  {
    prev_pressure_L[i] = normalPressure_L;
    prev_pressure_R[i] = normalPressure_R;
  }

  delay(5000);
}

float fake_pressure = 1;
float fake_airflow = 1.3;

void loop()
{
  //read data into variables
  // pressure_L = readPressure(pressurePin_L, normalPressure_L);
  airflow_L = readAirflow(len, airflowRawPin_L, airtempPin_L);
  pressure_R = readPressure(pressurePin_R, normalPressure_R);
  airflow_R = readAirflow(len, airflowRawPin_R, airtempPin_R) / fake_airflow;

  if (pressure_R > 1)
  {
    pressure_R = pressure_R * fake_pressure;
  }

  //average out when needed and calculate direction
  averageAndDirection(pressure_R, prev_pressure_R, direction_R);

  //calculate volume
  tidalVolume(airflow_R, volume_R, direction_R, 20);

  //send data through serial

  Serial.print(pressure_R);
  Serial.print(',');

  Serial.print(airflow_R * direction_R);
  Serial.print(',');

  dangerPressure(27);
  dangerAirflow(60);
  Serial.print(',');

  Serial.print(volume_R);
  Serial.print(',');

  dangerAirflow(6);

  Serial.print('\n');

  delay(20);
}