#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

uint8_t FHSShopInterval = 4;    

uint8_t  IntervalCount = 0;uint8_t  i = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
  Serial.print("txcallback");
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
  digitalToggle(PC13);
  Serial.println("RXdoneCallback");
  for (int i = 0; i < 8; i++)
  {
      Serial.print(Radio.RXdataBuffer[i], HEX);
      Serial.print(",");
  }
  Radio.RXnb();
  return true;
}

void SetRFLinkRate(uint8_t index) 
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
  // Binding always uses invertIQ
  bool invertIQ = InBindingMode || (UID[5] & 0x01);

  FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));


  Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);

  // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
  FHSSsetCurrIndex(0);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;

}

void setup()
{
  Serial.begin(115200);
  Serial.println("Begin SX1280 testing...");
  pinMode(PC13, OUTPUT);

  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.TXdoneCallback = &TXdoneCallback;
  Radio.RXdoneCallback = &RXdoneCallback;

  Radio.currFreq = FHSSgetInitialFreq(); //set frequency first or an error will occur!!!

  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  Radio.RXnb();
}

bool busy;
// 主循环
void loop()
{
  Serial.println("Current Index = " + String(FHSSgetCurrIndex()));
  // digitalWrite(PC13, HIGH);
  delay(500);
  // digitalWrite(PC13, LOW);
  // delay(500);
  busy = digitalRead(GPIO_PIN_BUSY);
  yield();
}
