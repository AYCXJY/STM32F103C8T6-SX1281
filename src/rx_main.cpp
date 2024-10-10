#include <Arduino.h>
#include <time.h>
#include "targets.h"
#include "common.h"
#include "SX1280Driver.h"
#include "FHSS.h"
#include "TimerInterrupt_Generic.h"
// OLED
#include <Adafruit_SSD1306.h>
#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(OLED_RESET);
// EEPROM
#include "elrs_eeprom.h"
ELRS_EEPROM eeprom;
// UID & BIND
#define UID_IS_BOUND(uid) (uid[2] != 255 || uid[3] != 255 || uid[4] != 255 || uid[5] != 255)
bool inBindingMode;
// FHSS
uint8_t FHSShopInterval = 4;    
uint8_t IntervalCount;
uint32_t currentFreq;
uint8_t currentchannel;
// Packet
#define PacketType_BIND   0
#define PacketType_DATA   1  
#define PacketType_SYNC   2  
#define payloadsize       5  

typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;

uint8_t rx_data;

uint32_t now;
uint16_t receivecount;
uint16_t receivefreq;

#define TIMER_INTERVAL_MS 1000000
STM32Timer ITimer(TIM1);
// 设置SX1280速率
void SetRFLinkRate(uint8_t index)
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
                ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
                uidMacSeedGet(), 0, 0);
}
// 退出绑定模式
void exitbindingmode(void)
{
    inBindingMode = false;
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
    currentchannel = 0;
    FHSSsetCurrIndex(currentchannel);
    currentFreq = FHSSgetInitialFreq();
    Radio.SetFrequencyReg(currentFreq);
    Radio.RXnb(SX1280_MODE_RX_CONT);

    FHSSrandomiseFHSSsequence(uidMacSeedGet());
}
// 进入绑定模式
void enterbindingmode(void)
{
    if(inBindingMode == false)
    {
        inBindingMode = true;
        SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
        currentchannel = 0;
        FHSSsetCurrIndex(currentchannel);
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
        Radio.RXnb(SX1280_MODE_RX_CONT);
    }
}
// 处理按键中断
void handleButtonPress() 
{
  enterbindingmode();
}
// 处理定时器中断 输出收包频率
void TimerHandler() 
{
  Serial.println(receivefreq);
  receivefreq = receivecount;
  receivecount = 0;
}
// 处理接收完毕中断
bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const status)
{
    Packet_t* const PktPtr = (Packet_t* const)(void*)Radio.RXdataBuffer;
    if(inBindingMode && PktPtr->type == PacketType_BIND)
    {
      memcpy(UID + 2, PktPtr->payload, 4);
      eeprom.Put(0, UID);
      eeprom.Commit();
      exitbindingmode();
    }
    else if (PktPtr->type == PacketType_DATA)
    {
      receivecount++;
      memcpy(&rx_data, PktPtr->payload, 1);
      currentchannel = PktPtr->currentchannel;
      IntervalCount = PktPtr->IntervalCount;
      if(IntervalCount == FHSShopInterval - 1)
      {
        currentFreq = FHSSgetNextFreq();
        Radio.SetFrequencyReg(currentFreq);
      }
    }
    return true;
}
// 初始化
void setup()
{
    // UART
    Serial.begin(115200);
    // LED
    pinMode(PC13, OUTPUT);
    // Button
    pinMode(PB1, INPUT_PULLUP);            
    attachInterrupt(digitalPinToInterrupt(PB1), handleButtonPress, FALLING);  
    // OLED
    Wire.setSCL(PB8);
    Wire.setSDA(PB9);
    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);            
    display.setTextColor(WHITE);        
    display.clearDisplay(); 
    display.display();
    // SX1280
    FHSSrandomiseFHSSsequence(uidMacSeedGet());
    pinMode(GPIO_PIN_TX_EN, OUTPUT);
    pinMode(GPIO_PIN_RX_EN, OUTPUT);
    Radio.RXdoneCallback = &RXdoneCallback;
    currentFreq = FHSSgetInitialFreq(); 
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
    // 开启连续接收模式
    Radio.RXnb(SX1280_MODE_RX_CONT);
    // eeprom
    eeprom.Begin();
    eeprom.Get(0, UID);
    if(!UID_IS_BOUND(UID))
    {
        enterbindingmode();
    }
    ITimer.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
}
// 主循环
void loop()
{
    display.clearDisplay();  
    if(inBindingMode)
    {            
        display.setCursor(0, 0);            
        display.println("receiving UID...");
    }
    else
    {
        // UID          
        display.setCursor(0, 0);           
        display.println("ID");         
        display.setCursor(18, 0);            
        display.println(UID[2]);
        display.setCursor(42, 0);            
        display.println(UID[3]);
        display.setCursor(66, 0);            
        display.println(UID[4]);
        display.setCursor(90, 0);            
        display.println(UID[5]);
    }
    // Freq
    display.setCursor(0, 8);           
    display.println("FQ");    
    display.setCursor(18, 8);           
    display.println(currentFreq);  
    // Channel
    display.setCursor(76, 8);           
    display.println("CH");  
    display.setCursor(94, 8);           
    display.println(currentchannel);  
    // Data 
    display.setCursor(0, 16);           
    display.println("Data");  
    display.setCursor(30, 16);           
    display.println(rx_data);  
    // Rate
    display.setCursor(0, 24);   
    display.println("Rate");     
    display.setCursor(30, 24);         
    display.println(receivefreq);   
    // RSSI
    display.setCursor(54, 24);     
    display.println("RSSI");     
    display.setCursor(84, 24);           
    display.println(Radio.GetRssiInst(SX12XX_Radio_1));  
    display.display();
}
