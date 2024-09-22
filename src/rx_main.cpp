#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

uint8_t FHSShopInterval = 4;    
uint8_t IntervalCount = 0;
uint8_t ACK = 0xAA;
uint8_t packetSize = 6;

// TX 中断回调函数
void ICACHE_RAM_ATTR TXdoneCallback()
{
  // 切换回接收模式
  Radio.RXnb(SX1280_MODE_RX_CONT);
  // 计算跳频
  IntervalCount++;
  if(IntervalCount % FHSShopInterval == 0)
  {
    Serial.println(FHSSgetNextFreq());
  }
}
// RX 中断回调函数
bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const status)
{
  // 如果接收到数据，LED 灯会和发送方同时闪烁。
  digitalToggle(PC13);
  // 打印 RXdataBuffer
  for (int i = 0; i < packetSize; i++){
    Serial.print((char)Radio.RXdataBuffer[i]);
  }
  Serial.println("");
  // 打印状态信息
  // Radio.GetStatus(SX12XX_Radio_1);
  // Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
  // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
  // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
  // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));
  Serial.println("Current Index = " + String(FHSSgetCurrIndex()));

  // 发送应答包
  Radio.TXnb(&ACK, 1, SX12XX_Radio_1);
  return true;
}
// 初始化空中速率
void SetRFLinkRate(uint8_t index){
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));
  ExpressLRS_currAirRate_Modparams = ModParams;
}

void setup(){
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
  // 开启连续接收模式
  Radio.RXnb(SX1280_MODE_RX_CONT);
}


void loop(){
  // 打印状态信息
  // Radio.GetStatus(SX12XX_Radio_1);
  // Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
  // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
  // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
  // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));

  yield();
}
