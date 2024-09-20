#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

bool InBindingMode = false;
volatile bool busyTransmitting;

// #define TX
#define RX

#if defined(TX)

uint8_t testdata[] = "HELLO!";
// every X packets we hop to a new frequency. Max value of 16 since only 4 bits have been assigned in the sync package.
uint8_t FHSShopInterval = 4;    
uint8_t  i = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
  i++;
  digitalToggle(PC13);

  Serial.println("HELLO!");
  Serial.println(FHSSgetCurrIndex());
  Serial.println(FHSSgetChannelCount());
  Serial.println(FHSSgetInitialFreq());
  if(i % 4 == 0){Serial.println(FHSSgetNextFreq());}
  Serial.println(FHSSgetSequenceCount());
  Serial.println(FHSSgetRegulatoryDomain());
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
  Serial.println("RXdoneCallback");
  for (int i = 0; i < 8; i++)
  {
      Serial.print(Radio.RXdataBuffer[i], HEX);
      Serial.print(",");
  }
  Serial.println("");
  Radio.RXnb();
  return true;
}

void SetRFLinkRate(uint8_t index) // Set speed of RF link
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

  // Radio.SetFrequencyReg(FHSSgetInitialFreq());

  Radio.currFreq = FHSSgetInitialFreq(); //set frequency first or an error will occur!!!

  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);
}

// 主循环
void loop()
{
  delay(500);
  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_All);
  // Radio.RXnb();
  yield();
}
#endif
/******************************************* */


/*************************************** */
#if defined(RX)
uint8_t FHSShopInterval = 4;    
uint8_t  i = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{

}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
  Serial.println("RXdoneCallback");
  for (int i = 0; i < 8; i++)
  {
      Serial.print(Radio.RXdataBuffer[i], HEX);
      Serial.print(",");
  }
  Serial.println("");
  Radio.RXnb();
  return true;
}

void SetRFLinkRate(uint8_t index) // Set speed of RF link
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

  // Radio.SetFrequencyReg(FHSSgetInitialFreq());

  Radio.currFreq = FHSSgetInitialFreq(); //set frequency first or an error will occur!!!

  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  Radio.RXnb();
}

// 主循环
void loop()
{

  yield();
}
#endif