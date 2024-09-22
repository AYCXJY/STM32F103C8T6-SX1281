#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

uint8_t testdata[] = "HELLO!";
// uint8_t FHSShopInterval = 4;    
// uint8_t IntervalCount = 0;


void ICACHE_RAM_ATTR TXdoneCallback()
{
  // IntervalCount++;
  // if(IntervalCount % 4 == 0)
  // {
  //   Serial.println(FHSSgetNextFreq());
  // }
  // 发送完数据翻转LED
  digitalToggle(PC13);
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
  return true;
}

// 初始化空中速率
void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));

  ExpressLRS_currAirRate_Modparams = ModParams;
}

void setup()
{
  Serial.begin(115200);
  // 发送和接收使能位
  pinMode(GPIO_PIN_TX_EN, OUTPUT);
  pinMode(GPIO_PIN_RX_EN, OUTPUT);
  // LED
  pinMode(PC13, OUTPUT);
  // 初始化跳频随机序列
  FHSSrandomiseFHSSsequence(uidMacSeedGet());

  Radio.TXdoneCallback = &TXdoneCallback;
  Radio.RXdoneCallback = &RXdoneCallback;

  Radio.currFreq = FHSSgetInitialFreq(); 

  // RF初始化
  bool init_success;
  init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  if(!init_success){Serial.println("Radio.Begin failed!");}
  SetRFLinkRate(9);
}

void loop()
{
  Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_1);


  // 打印状态信息
  Radio.GetStatus(SX12XX_Radio_1);
  // 发送模式时RSSI恒为-127
  Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
  // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
  // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
  // Serial.println("Current Index = " + String(FHSSgetCurrIndex()));
  // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));


  delay(1000);
  yield();
}


