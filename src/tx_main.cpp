#include <Arduino.h>
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
// send rate
uint16_t sendcount;
uint16_t sendfreq;
// 1s timer
#define TIMER_INTERVAL_MS 1000000
STM32Timer ITimer(TIM1);
// Packet
#define PacketType_BIND   0
#define PacketType_DATA   1  
#define PacketType_SYNC   2  
#define payloadsize       5
WORD_ALIGNED_ATTR typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;
Packet_t packet;
// 1byte tx data
uint8_t tx_data;
// indicate TXBusyStatus
volatile bool busyTransmitting;
// BIND
bool inBindingMode;
uint8_t bindcount;
// FHSS
uint8_t FHSShopInterval = 4;    
uint8_t IntervalCount = 0;
uint32_t currentFreq = 0;

// 设置SX1280速率
void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, 0);
}
// 从MCU获得UID
static void setupBindingFromConfig()
{
  UID[0] = (uint8_t)HAL_GetUIDw0();
  UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
  UID[2] = (uint8_t)HAL_GetUIDw1();
  UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
  UID[4] = (uint8_t)HAL_GetUIDw2();
  UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);
}
// 退出绑定模式
void exitbindingmode(void)
{
    inBindingMode = false;  
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
    FHSSsetCurrIndex(0);
    currentFreq = FHSSgetInitialFreq();
    Radio.SetFrequencyReg(currentFreq);
}
// 进入绑定模式
void enterbindingmode(void)
{
    if(inBindingMode == false)
    {
        inBindingMode = true;
        SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
        FHSSsetCurrIndex(0);
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
    }   
}
// 处理按键中断
void handleButtonPress()
{
  enterbindingmode();
}
// 处理定时器中断 输出发包频率
void TimerHandler() 
{
  Serial.println(sendfreq);
  sendfreq = sendcount;
  sendcount = 0;
}
// 处理发送完毕中断
void ICACHE_RAM_ATTR TXdoneCallback()
{
    sendcount++;
    busyTransmitting = false;
    if(inBindingMode == false)
    {
        IntervalCount++;
        if(IntervalCount % FHSShopInterval == 0)
        {
            IntervalCount = 0;
            currentFreq = FHSSgetNextFreq();
            Radio.SetFrequencyReg(currentFreq);
        }
    }
    else {bindcount++;}
}
// 初始化
void setup()
{
    // UART
    Serial.begin(420000);
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
    setupBindingFromConfig();
    FHSSrandomiseFHSSsequence(uidMacSeedGet());
    pinMode(GPIO_PIN_TX_EN, OUTPUT);
    pinMode(GPIO_PIN_RX_EN, OUTPUT);
    Radio.TXdoneCallback = &TXdoneCallback;
    currentFreq = FHSSgetInitialFreq(); 
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
    // timer
    ITimer.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
    // OLED display
    display.clearDisplay();  
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
    display.display();
}
// 主循环
void loop()
{
    if(inBindingMode)
    { 
        // load
        packet.type = PacketType_BIND;
        packet.IntervalCount = IntervalCount;
        packet.currentchannel = FHSSgetCurrIndex();
        packet.payloadSize = sizeof(UID) - 2;
        memcpy(packet.payload, UID + 2, packet.payloadSize);
        // send
        bindcount = 0;
        while(bindcount < 50)
        {
            busyTransmitting = true;
            Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
            while(busyTransmitting){yield();}
        }
        // exit
        exitbindingmode();
    }
    else
    {
        // load
        packet.type = PacketType_DATA;
        packet.IntervalCount = IntervalCount;
        packet.currentchannel = FHSSgetCurrIndex();
        packet.payloadSize = 1;
        memcpy(packet.payload, &tx_data, packet.payloadSize);
        tx_data++;
        // send
        busyTransmitting = true;
        Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
        while(!inBindingMode && busyTransmitting){yield();}
    }
}


