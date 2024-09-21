#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

uint8_t testdata[] = "HELLO!";

// every X packets we hop to a new frequency. 
// Max value of 16 since only 4 bits have been assigned in the sync package.
uint8_t FHSShopInterval = 4;    

uint8_t  IntervalCount = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
  // IntervalCount++;
  // if(IntervalCount % 4 == 0){Serial.println(FHSSgetNextFreq());}

  digitalToggle(PC13);

  Serial.println("Current Index = " + String(FHSSgetCurrIndex()));
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
  return true;
}

void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

  bool invertIQ = InBindingMode || (UID[5] & 0x01);

  FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && 
                            !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
  FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));


  Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : 
                            (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);

  // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
  FHSSsetCurrIndex(0);

  ExpressLRS_currAirRate_Modparams = ModParams;
  ExpressLRS_currAirRate_RFperfParams = RFperf;

}

void setup()
{
  Serial.begin(115200);
  pinMode(PC13, OUTPUT);

  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.TXdoneCallback = &TXdoneCallback;
  Radio.RXdoneCallback = &RXdoneCallback;

  Radio.currFreq = FHSSgetInitialFreq(); 

  bool init_success;
  init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  if(!init_success){Serial.println("Radio.Begin failed!");}

  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);
}

bool busy;


void loop()
{
  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);
  delay(100);

  busy = digitalRead(GPIO_PIN_BUSY);
  Serial.println(Radio.GetRssiInst(SX12XX_Radio_All));
  Radio.GetStatus(SX12XX_Radio_All);

  yield();
}


