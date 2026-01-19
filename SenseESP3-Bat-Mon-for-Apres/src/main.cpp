/*
 * Apres 3 Battery Monitor using SensESP V3
 * Copyright (c) 2026 Jason Greenwood 
 * https://www.youtube.com/@ApresSail
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
// Apres 3 Battery Battery Monitor using SensESP V3.
// Software version 3.2.0 - 20260118
// Changes for version 3.2.0
// 1. Change SOC for Battery B to LiFePo4 - complete 
// 2. Add AC Current Sensors x2 
//  a. Inverter
//  b. Shore Power
// 3. 4th INA219 for fuel level sensor - Complete
//
#include <memory>
#include "sensesp.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include <Wire.h>  
#include <Adafruit_INA219.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

using namespace sensesp;
using namespace sensesp::onewire;

// 1) Direct copy from old Main.ccp - Start
  // State of Charge lookup
class SoCAInterpreter : public CurveInterpolator {
 public:
  SoCAInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the Battery A (Start) Voltage values returned by
    // our INA219_A to % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownVoltage, knownSOC%));
    add_sample(CurveInterpolator::Sample(2.5, 0));
    add_sample(CurveInterpolator::Sample(10.5, 0));
    add_sample(CurveInterpolator::Sample(11.31, .1));
    add_sample(CurveInterpolator::Sample(11.58, .2));
    add_sample(CurveInterpolator::Sample(11.75, .3));
    add_sample(CurveInterpolator::Sample(11.9, .4));
    add_sample(CurveInterpolator::Sample(12.06, .5));
    add_sample(CurveInterpolator::Sample(12.2, .6));
    add_sample(CurveInterpolator::Sample(12.32, .7));
    add_sample(CurveInterpolator::Sample(12.42, .8)); 
    add_sample(CurveInterpolator::Sample(12.5, .9));
    add_sample(CurveInterpolator::Sample(12.6, 1)); 
    add_sample(CurveInterpolator::Sample(20.0, 1)); 
  }
};
class SoCBInterpreter : public CurveInterpolator {
 public:
  SoCBInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the Battery B (LiFePo4) Voltage values returned by
    // our INA219_B to % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownVoltage, knownSOC%));
    add_sample(CurveInterpolator::Sample(2.5, 0));
    add_sample(CurveInterpolator::Sample(11.200, 0));
    add_sample(CurveInterpolator::Sample(12.88, .1));
    add_sample(CurveInterpolator::Sample(13.064, .2));
    add_sample(CurveInterpolator::Sample(13.12, .3));
    add_sample(CurveInterpolator::Sample(13.164, .4));
    add_sample(CurveInterpolator::Sample(13.204, .5));
    add_sample(CurveInterpolator::Sample(13.288, .6));
    add_sample(CurveInterpolator::Sample(13.30, .7));
    add_sample(CurveInterpolator::Sample(113.316, .8)); 
    add_sample(CurveInterpolator::Sample(13.32, .9));
    add_sample(CurveInterpolator::Sample(13.4, 1)); 
    add_sample(CurveInterpolator::Sample(20.0, 1)); 
  }
};
class SoCCInterpreter : public CurveInterpolator {
 public:
  SoCCInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the Battery C Voltage values returned by
    // our INA219_C to % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownVoltage, knownSOC%));
    add_sample(CurveInterpolator::Sample(2.5, 0));
    add_sample(CurveInterpolator::Sample(10.5, 0));
    add_sample(CurveInterpolator::Sample(11.31, .1));
    add_sample(CurveInterpolator::Sample(11.58, .2));
    add_sample(CurveInterpolator::Sample(11.75, .3));
    add_sample(CurveInterpolator::Sample(11.9, .4));
    add_sample(CurveInterpolator::Sample(12.06, .5));
    add_sample(CurveInterpolator::Sample(12.2, .6));
    add_sample(CurveInterpolator::Sample(12.32, .7));
    add_sample(CurveInterpolator::Sample(12.42, .8)); 
    add_sample(CurveInterpolator::Sample(12.5, .9));
    add_sample(CurveInterpolator::Sample(12.6, 1)); 
    add_sample(CurveInterpolator::Sample(20.0, 1)); 
  }
};
class FuelLevelInterpreter : public CurveInterpolator {
 public:
  FuelLevelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the gauge voltage values returned by
    // our INA219_C to Fuel Level % (Ratio)
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhm, knownFuelLevel)); - these numbers need changing
    add_sample(CurveInterpolator::Sample(1000, 0));
    add_sample(CurveInterpolator::Sample(240, 0));
    add_sample(CurveInterpolator::Sample(218, .1));
    add_sample(CurveInterpolator::Sample(198, .2));
    add_sample(CurveInterpolator::Sample(177, .3));
    add_sample(CurveInterpolator::Sample(156, .4));
    add_sample(CurveInterpolator::Sample(135, .5));
    add_sample(CurveInterpolator::Sample(114, .6));
    add_sample(CurveInterpolator::Sample(93, .7));
    add_sample(CurveInterpolator::Sample(72, .8)); 
    add_sample(CurveInterpolator::Sample(51, .9));
    add_sample(CurveInterpolator::Sample(30, 1)); 
    add_sample(CurveInterpolator::Sample(0, 1)); 
  }
};
////////////////////INA219
  Adafruit_INA219 ina219_A;
  Adafruit_INA219 ina219_B(0x41);
  Adafruit_INA219 ina219_C(0x44);
  Adafruit_INA219 ina219_d(0x45);

  const float RshuntA = 0.0005;
  const float RshuntB = 0.0005;
  const float RshuntC = 0.0005;

  //float read_A_current_callback() { return (ina219_A.getCurrent_mA() / 1000);}
  float read_A_current_callback() { return ((ina219_A.getShuntVoltage_mV() / 1000) / RshuntA);}
  float read_A_shuntvoltage_callback() { return (ina219_A.getShuntVoltage_mV() / 1000);}
  float read_A_busvoltage_callback() { return (ina219_A.getBusVoltage_V());}
  float read_A_loadvoltage_callback() { return (ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000));}
  float read_A_power_callback() { return ((ina219_A.getBusVoltage_V() + (ina219_A.getShuntVoltage_mV() / 1000)) * (ina219_A.getCurrent_mA() / 1000));}

    //float read_B_current_callback() { return (ina219_B.getCurrent_mA() / 1000);}
  float read_B_current_callback() { return ((ina219_B.getShuntVoltage_mV() / 1000) / RshuntB);}
  float read_B_shuntvoltage_callback() { return (ina219_B.getShuntVoltage_mV() / 1000);}
  float read_B_busvoltage_callback() { return (ina219_B.getBusVoltage_V());}
  float read_B_loadvoltage_callback() { return (ina219_B.getBusVoltage_V() + (ina219_B.getShuntVoltage_mV() / 1000));}
  float read_B_power_callback() { return ((ina219_B.getBusVoltage_V() + (ina219_B.getShuntVoltage_mV() / 1000)) * (ina219_B.getCurrent_mA() / 1000));}

    //float read_C_current_callback() { return (ina219_C.getCurrent_mA() / 1000);}
  float read_C_current_callback() { return ((ina219_C.getShuntVoltage_mV() / 1000) / RshuntC);}
  float read_C_shuntvoltage_callback() { return (ina219_C.getShuntVoltage_mV() / 1000);}
  float read_C_busvoltage_callback() { return (ina219_C.getBusVoltage_V());}
  float read_C_loadvoltage_callback() { return (ina219_C.getBusVoltage_V() + (ina219_C.getShuntVoltage_mV() / 1000));}
  float read_C_power_callback() { return ((ina219_C.getBusVoltage_V() + (ina219_C.getShuntVoltage_mV() / 1000)) * (ina219_C.getCurrent_mA() / 1000));}

  float read_D_shuntvoltage_callback() { return (ina219_d.getShuntVoltage_mV() / 1000);} //drop across the shunt
  float read_D_busvoltage_callback() { return (ina219_d.getBusVoltage_V());} //downstream voltage to be supplied to the consumer
  float read_D_loadvoltage_callback() { return (ina219_d.getBusVoltage_V() + (ina219_d.getShuntVoltage_mV() / 1000));} //loadvoltage is the battery or supply voltage
//////////////////////INA219 end
///////////////////AC Sensor declaration
Adafruit_ADS1115 ads_ac;  // AC current ADC

// NEW (for your SCT-013-030 voltage-output):
const float CT_MAX_PRIMARY_AMPS = 30.0f;    // datasheet max
const float CT_MAX_OUTPUT_VOLTS = 1.0f;     // datasheet max
const float ADS_REF_VOLT       = 4.096f;    // because GAIN_ONE
const float ADS_LSB_VOLT       = 0.000125f; // 4.096 / 32768 ~ 125uV

const int   AC_SAMPLE_COUNT    = 2000;      // ~40 ms at 50kS/s; tune to match sketch
const int   AC_SAMPLE_DELAY_US = 200;       // per-sample delay to get desired sample rate

float calc_ac_rms_from_ads_channel(uint8_t channel) {
  long sum_sq = 0;
  for (int i = 0; i < AC_SAMPLE_COUNT; i++) {
    int16_t raw = ads_ac.readADC_SingleEnded(channel);
    // Remove DC offset (mid-scale = 0V AC)
    float centered_raw = raw - 2048;  // 12-bit ADC midpoint
    sum_sq += (long)centered_raw * (long)centered_raw;
    delayMicroseconds(AC_SAMPLE_DELAY_US);
  }
  
  float mean_sq = (float)sum_sq / (float)AC_SAMPLE_COUNT;
  float rms_adc_counts = sqrtf(mean_sq);
  float rms_volts_adc = rms_adc_counts * ADS_LSB_VOLT;
  
  // Convert ADC volts directly to primary current
  // I_primary = (V_ct_rms / V_ct_max) * I_primary_max
  float ct_voltage_ratio = rms_volts_adc / CT_MAX_OUTPUT_VOLTS;
  float I_primary_amps = ct_voltage_ratio * CT_MAX_PRIMARY_AMPS;
  
  return I_primary_amps;
}

// Inverter AC channel on ADS1115 A0
float read_inverter_ac_current_callback() {
  return calc_ac_rms_from_ads_channel(0);
}
// Shore AC channel on ADS1115 A1
float read_shore_ac_current_callback() {
  return calc_ac_rms_from_ads_channel(1);
}
////AC Sensor End
///////////////////Blackwater sensors start
float blackwaterlevel = 0;
bool black5_state = false;
bool black4_state = false;
bool black3_state = false;
bool black2_state = false;
bool black1_state = false;
float blackwaterlevel_callback() { return (blackwaterlevel);}

void blackwater_calc() {

if (black5_state == 1) {
  blackwaterlevel = 1;
} else if (black4_state == 1) {
  blackwaterlevel = 0.8;
} else if (black3_state == 1) {
  blackwaterlevel = 0.6;
} else if (black2_state == 1) {
  blackwaterlevel = 0.4;
} else if (black1_state == 1) {
  blackwaterlevel = 0.2;
} else {
  blackwaterlevel = 0.0;
}
}
//////////////////Blackwater sensors End
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  Wire.begin(21,22);                // join i2c bus (address optional for master)

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Apres-Bat-Mon-V3.2.0")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi_client("My WiFi SSID", "my_wifi_password")
                    //->set_wifi_access_point("My AP SSID", "my_ap_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

 // 3) Direct copy from old Main.ccp - Start 
/// 1-Wire Temp Sensors (amended for SensESP version 3)

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // A Battery Temperature - electrical.batteries.A.temperature
  auto* A_bat_temp =
      new OneWireTemperature(dts, 1000, "/A Battery Temperature/oneWire");

  ConfigItem(A_bat_temp)
      ->set_title("A Battery Temperature")
      ->set_description("Temperature of the A Battery")
      ->set_sort_order(100);
    
  auto A_bat_temp_calibration =
      new Linear(1.0, 0.0, "/A Battery Temperature/linear");
      
  ConfigItem(A_bat_temp_calibration)
      ->set_title("A Battery Temperature Calibration")
      ->set_description("Calibration for the A Battery temperature sensor")
      ->set_sort_order(200);

  auto A_bat_temp_sk_output = new SKOutputFloat(
      "electrical.batteries.A.temperature", "/A Battery Temperature/skPath");

  ConfigItem(A_bat_temp_sk_output)
      ->set_title("A Battery Temperature Signal K Path")
      ->set_description("Signal K path for the A Battery temperature")
      ->set_sort_order(300);

  A_bat_temp->connect_to(A_bat_temp_calibration)
      ->connect_to(A_bat_temp_sk_output);

 // B Battery Temperature - electrical.batteries.B.temperature
   auto* B_bat_temp =
      new OneWireTemperature(dts, 1000, "/B Battery Temperature/oneWire");

  ConfigItem(B_bat_temp)
      ->set_title("B Battery Temperature")
      ->set_description("Temperature of the B Battery")
      ->set_sort_order(100);
    
  auto B_bat_temp_calibration =
      new Linear(1.0, 0.0, "/B Battery Temperature/linear");
      
  ConfigItem(B_bat_temp_calibration)
      ->set_title("B Battery Temperature Calibration")
      ->set_description("Calibration for the B Battery temperature sensor")
      ->set_sort_order(200);

  auto B_bat_temp_sk_output = new SKOutputFloat(
      "electrical.batteries.B.temperature", "/B Battery Temperature/skPath");

  ConfigItem(B_bat_temp_sk_output)
      ->set_title("B Battery Temperature Signal K Path")
      ->set_description("Signal K path for the B Battery temperature")
      ->set_sort_order(300);

  B_bat_temp->connect_to(B_bat_temp_calibration)
      ->connect_to(B_bat_temp_sk_output); 

  // C Battery Temperature - electrical.batteries.C.temperature
  auto* C_bat_temp =
      new OneWireTemperature(dts, 1000, "/C Battery Temperature/oneWire");

  ConfigItem(C_bat_temp)
      ->set_title("C Battery Temperature")
      ->set_description("Temperature of the C Battery")
      ->set_sort_order(100);
    
  auto C_bat_temp_calibration =
      new Linear(1.0, 0.0, "/C Battery Temperature/linear");
      
  ConfigItem(C_bat_temp_calibration)
      ->set_title("C Battery Temperature Calibration")
      ->set_description("Calibration for the C Battery temperature sensor")
      ->set_sort_order(200);

  auto C_bat_temp_sk_output = new SKOutputFloat(
      "electrical.batteries.A.temperature", "/C Battery Temperature/skPath");

  ConfigItem(C_bat_temp_sk_output)
      ->set_title("C Battery Temperature Signal K Path")
      ->set_description("Signal K path for the C Battery temperature")
      ->set_sort_order(300);

  C_bat_temp->connect_to(C_bat_temp_calibration)
      ->connect_to(C_bat_temp_sk_output);

///////////////////////////INA219 start
  ina219_A.begin();  // Initialize first board (default address 0x40)
  ina219_B.begin();  // Initialize second board with the address 0x41 ----> A0 soldered = 0x41
  ina219_C.begin();  // Initialize second board with the address 0x44 ----> A1 soldered = 0x44
  ina219_d.begin();  // Initialize second board with the address 0x45 ----> A1 and A0 soldered = 0x45

//ina219_A - Start Battery
  auto* ina219_A_current =
      new RepeatSensor<float>(1000, read_A_current_callback);

  auto* ina219_A_shuntvoltage = 
      new RepeatSensor<float>(1000, read_A_shuntvoltage_callback);

  auto* ina219_A_busvoltage = 
      new RepeatSensor<float>(1000, read_A_busvoltage_callback);   

  auto* ina219_A_loadvoltage = 
      new RepeatSensor<float>(1000, read_A_loadvoltage_callback); 

    auto* ina219_A_power = 
      new RepeatSensor<float>(1000, read_A_power_callback);    

  // Send the temperature to the Signal K server as a Float
  ina219_A_current->connect_to(new SKOutputFloat("electrical.batteries.A.current"));

  ina219_A_loadvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.voltage"));

  //ina219_A_shuntvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.shuntv"));

  //ina219_A_busvoltage->connect_to(new SKOutputFloat("electrical.batteries.A.vin-v"));

  ina219_A_power->connect_to(new SKOutputFloat("electrical.batteries.A.power"));

  ina219_A_loadvoltage->connect_to(new SoCAInterpreter("/Battery A/Voltage/curve"))
      ->connect_to(new SKOutputFloat("electrical.batteries.A.capacity.stateOfCharge", "/Battery A State of Charge/sk_path"));


//ina219_B - House 1 Battery
  auto* ina219_B_current =
      new RepeatSensor<float>(1000, read_B_current_callback);

  auto* ina219_B_shuntvoltage = 
      new RepeatSensor<float>(1000, read_B_shuntvoltage_callback);

  auto* ina219_B_busvoltage = 
      new RepeatSensor<float>(1000, read_B_busvoltage_callback);   

  auto* ina219_B_loadvoltage = 
      new RepeatSensor<float>(1000, read_B_loadvoltage_callback); 

    auto* ina219_B_power = 
      new RepeatSensor<float>(1000, read_B_power_callback);    

  // Send the temperature to the Signal K server as a Float
  ina219_B_current->connect_to(new SKOutputFloat("electrical.batteries.B.current"));

  ina219_B_loadvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.voltage"));

  //ina219_B_shuntvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.shuntv"));

  //ina219_B_busvoltage->connect_to(new SKOutputFloat("electrical.batteries.B.vin-v"));

  ina219_B_power->connect_to(new SKOutputFloat("electrical.batteries.B.power"));

  ina219_B_loadvoltage->connect_to(new SoCBInterpreter("/Battery B/Voltage/curve"))
      ->connect_to(new SKOutputFloat("electrical.batteries.B.capacity.stateOfCharge", "/Battery B State of Charge/sk_path"));

  //ina219_C
  auto* ina219_C_current =
      new RepeatSensor<float>(1000, read_C_current_callback);

  auto* ina219_C_shuntvoltage = 
      new RepeatSensor<float>(1000, read_C_shuntvoltage_callback);

  auto* ina219_C_busvoltage = 
      new RepeatSensor<float>(1000, read_C_busvoltage_callback);   

  auto* ina219_C_loadvoltage = 
      new RepeatSensor<float>(1000, read_C_loadvoltage_callback); 

    auto* ina219_C_power = 
      new RepeatSensor<float>(1000, read_C_power_callback);    

  // Send the temperature to the Signal K server as a Float
  ina219_C_current->connect_to(new SKOutputFloat("electrical.batteries.C.current"));

  ina219_C_loadvoltage->connect_to(new SKOutputFloat("electrical.batteries.C.voltage"));

  //ina219_C_shuntvoltage->connect_to(new SKOutputFloat("electrical.batteries.C.shuntv"));

  //ina219_C_busvoltage->connect_to(new SKOutputFloat("electrical.batteries.C.vin-v"));

  ina219_C_power->connect_to(new SKOutputFloat("electrical.batteries.C.power"));

  ina219_C_loadvoltage->connect_to(new SoCCInterpreter("/Battery C/Voltage/curve"))
        ->connect_to(new SKOutputFloat("electrical.batteries.C.capacity.stateOfCharge", "/Battery C State of Charge/sk_path"));

  //ina219_D
  //// Fuel Sender Config ////

  auto* ina219_D_shuntvoltage = 
      new RepeatSensor<float>(1000, read_D_shuntvoltage_callback); //drop across the shunt

  auto* ina219_D_busvoltage = 
      new RepeatSensor<float>(1000, read_D_busvoltage_callback);  //downstream voltage 

  auto* ina219_D_loadvoltage = 
      new RepeatSensor<float>(1000, read_D_loadvoltage_callback);   //battery voltage

  // Send the temperature to the Signal K server as a Float

  ina219_D_loadvoltage->connect_to(new SKOutputFloat("tanks.fuel.diesel.Vin.voltage")); 

  ina219_D_shuntvoltage->connect_to(new SKOutputFloat("tanks.fuel.diesel.differential.voltage")); 

  ina219_D_busvoltage->connect_to(new SKOutputFloat("tanks.fuel.diesel.Vout.voltage")); 

const float R1  = 47.0;   // analogue gauge resistance in ohms
const float Vin = 14.0;   // approximate battery / gauge supply voltage

ina219_D_busvoltage
    ->connect_to(new VoltageDividerR2(R1, Vin, "/tanks/fuel/sender"))
    ->connect_to(new FuelLevelInterpreter("/tanks/fuel/curve"))
    ->connect_to(new Linear(1.0, 0.0, "/tanks/fuel/calibrate"))
    ->connect_to(new SKOutputFloat("tanks.fuel.diesel.currentlevel",
                                   "/tanks/fuel/sk_path"));
//////////////////AC Sensor
  ads_ac.setGain(GAIN_ONE);      // +/-4.096V range, adjust to your CT/burden
  ads_ac.begin(0x48);           // default ADS1115 address; change if needed
//
  // AC Current - Inverter
  auto* inverter_ac_current =
      new RepeatSensor<float>(2000, read_inverter_ac_current_callback); // every 2 s

  // AC Current - Shore Power
  auto* shore_ac_current =
      new RepeatSensor<float>(2000, read_shore_ac_current_callback);
  
  ////SignalK outputs
      inverter_ac_current->connect_to(
      new SKOutputFloat("electrical.ac.inverter.current",
                        "/AC/Inverter/current"));

  shore_ac_current->connect_to(
      new SKOutputFloat("electrical.ac.shore.current",
                        "/AC/Shore/current"));
////////////AC Sensor end
auto* black5_input_state = new DigitalInputState(2, INPUT_PULLUP, 1000); //Digital 9
black5_input_state->connect_to(
      new LambdaConsumer<bool>([](bool state5) {
        black5_state = state5;
      }));

auto* black4_input_state = new DigitalInputState(13, INPUT_PULLUP, 1000); //Digital 7
black4_input_state->connect_to(
      new LambdaConsumer<bool>([](bool state4) {
        black4_state = state4;
      }));

auto* black3_input_state = new DigitalInputState(14, INPUT_PULLUP, 1000); //Digital 6
black3_input_state->connect_to(
      new LambdaConsumer<bool>([](float state3) {
        black3_state = state3;
        blackwater_calc();
      })); 

auto* black2_input_state = new DigitalInputState(0, INPUT_PULLUP, 1000); //Digital 5
black2_input_state->connect_to(
      new LambdaConsumer<bool>([](float state2) {
        black2_state = state2;
      })); 

auto* black1_input_state = new DigitalInputState(26, INPUT_PULLUP, 1000); //Digital 3
black1_input_state->connect_to(
      new LambdaConsumer<bool>([](float state1) {
        black1_state = state1;
      }));             

  auto* blackwater_level = 
      new RepeatSensor<float>(1000, blackwaterlevel_callback); 

blackwater_level->connect_to(new SKOutputFloat("tanks.blackwater.1.currentLevel"));
black1_input_state->connect_to(new SKOutputFloat("tanks.black_state.1.currentLevel"));
black2_input_state->connect_to(new SKOutputFloat("tanks.black_state.2.currentLevel"));
black3_input_state->connect_to(new SKOutputFloat("tanks.black_state.3.currentLevel"));
black4_input_state->connect_to(new SKOutputFloat("tanks.black_state.4.currentLevel"));
black5_input_state->connect_to(new SKOutputFloat("tanks.black_state.5.currentLevel"));
//
  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
