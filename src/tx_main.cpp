#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

volatile bool busyTransmitting;

uint8_t testdata[] = "HELLO!";

// every X packets we hop to a new frequency. 
// Max value of 16 since only 4 bits have been assigned in the sync package.
uint8_t FHSShopInterval = 4;    

uint8_t  IntervalCount = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
  IntervalCount++;
  if(IntervalCount % 4 == 0){Serial.println(FHSSgetNextFreq());}

  digitalToggle(PC13);

  Serial.println("Current Index:" + String(FHSSgetCurrIndex()));
  Serial.println("Channel Count:" + String(FHSSgetChannelCount()));
  Serial.println("Initial Freq:" + String(FHSSgetInitialFreq()));
  Serial.println("Sequense Count:" + String(FHSSgetSequenceCount()));
  Serial.println("Regulatory Domain:" + String(FHSSgetRegulatoryDomain()));

  if (!busyTransmitting)
  {
    return; // Already finished transmission and do not call HandleFHSS() a second time, which may hop the frequency!
  }
  // if (connectionState != awaitingModelId)
  // {
  //   HandleFHSS();
  //   HandlePrepareForTLM();
  //   #if defined(Regulatory_Domain_EU_CE_2400)
  //   if (TelemetryRcvPhase != ttrpPreReceiveGap)
  //   {
  //   // Start RX for Listen Before Talk early because it takes about 100us
  //   // from RX enable to valid instant RSSI values are returned.
  //   // If rx was already started by TLM prepare above, this call will let RX
  //   // continue as normal.
  //   SetClearChannelAssessmentTime();
  //   }
  //   #endif // non-CE
  // }
  busyTransmitting = false;
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
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
  pinMode(PC13, OUTPUT);

  // 配置GPIO引脚模式
  pinMode(GPIO_PIN_NSS, OUTPUT);
  pinMode(GPIO_PIN_MOSI, OUTPUT);
  pinMode(GPIO_PIN_MISO, INPUT);
  pinMode(GPIO_PIN_SCK, OUTPUT);

  pinMode(GPIO_PIN_DIO1, INPUT);
  pinMode(GPIO_PIN_RST, OUTPUT);
  pinMode(GPIO_PIN_BUSY, INPUT);

  pinMode(GPIO_PIN_RCSIGNAL_RX, INPUT);
  pinMode(GPIO_PIN_RCSIGNAL_TX, OUTPUT);

  pinMode(GPIO_PIN_LED_RED, OUTPUT);

  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.TXdoneCallback = &TXdoneCallback;
  Radio.RXdoneCallback = &RXdoneCallback;

  Radio.currFreq = FHSSgetInitialFreq(); 

  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);
}

void loop()
{
  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);

  delay(500);

  yield();
}


