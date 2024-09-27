#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

#include <Adafruit_SSD1306.h>
#define OLED_RESET     4 // 定义 OLED 的复位引脚
#define SCREEN_WIDTH   128 // OLED 屏幕宽度
#define SCREEN_HEIGHT  64  // OLED 屏幕高度
// 初始化 OLED 显示器
Adafruit_SSD1306 display(OLED_RESET);

uint8_t FHSShopInterval = 4;    
uint8_t IntervalCount = 0;
uint32_t currentFreq;
uint8_t ACK = 0x00;
uint8_t packetSize = 6;

static void setupBindingFromConfig()
{
  // if (firmwareOptions.hasUID)
  // {
  //     memcpy(UID, firmwareOptions.uid, UID_LEN);
  // }
  // else
  {
#ifdef PLATFORM_ESP32
    esp_read_mac(UID, ESP_MAC_WIFI_STA);
#elif PLATFORM_STM32
    UID[0] = (uint8_t)HAL_GetUIDw0();
    UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
    UID[2] = (uint8_t)HAL_GetUIDw1();
    UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
    UID[4] = (uint8_t)HAL_GetUIDw2();
    UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);
#endif
  }

  Serial.print("UID = ");
  for(int i = 5; i >= 0; i--)  
  {
    Serial.print(UID[i]);
  }
}


// TX ACK中断回调函数
void ICACHE_RAM_ATTR TXdoneCallback()
{
  // 计算跳频
  ACK++;
  IntervalCount++;
  if(IntervalCount % FHSShopInterval == 0)
  {
    IntervalCount = 0;
    currentFreq = FHSSgetNextFreq();
    Radio.SetFrequencyReg(currentFreq);
  }
  // 切换回接收模式
  Radio.RXnb(SX1280_MODE_RX_CONT);
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
  // Serial.println("Current Index = " + String(FHSSgetCurrIndex()));
    Serial.println(currentFreq);

    display.clearDisplay();             // 清除屏幕
    display.setCursor(0, 0);            // 设置光标位置
    display.println(Radio.GetRssiInst(SX12XX_Radio_1));    // 显示文本
    display.setCursor(12, 16);            // 设置光标位置
    display.println(currentFreq);       // 显示文本
    display.display();
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

void setup()
{
  // 初始化 I2C 总线
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
    // 初始化 OLED 显示器
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // OLED 地址通常为 0x3C 或 0x3D
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // 不成功则停止
  }
  display.setTextSize(2);             // 正常文本大小
  display.setTextColor(WHITE);        // 文本颜色为白色
  display.clearDisplay(); // 清除屏幕
  display.display();

  setupBindingFromConfig();

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


void loop()
{
  // 打印状态信息
  // Radio.GetStatus(SX12XX_Radio_1);
  // Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
  // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
  // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
  // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));
  display.clearDisplay();             // 清除屏幕
  display.setCursor(0, 0);            // 设置光标位置
  display.println(UID[0]);    // 显示文本
  display.display();
  digitalToggle(PC13);
  delay(100); 
  yield();
}
