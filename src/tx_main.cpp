#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "targets.h"
#include "common.h"
#include "SX1280Driver.h"
#include "FHSS.h"
#include "hwTimer.h"

/******************* define *********************/
// oled setting
#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64
// packet type
#define PacketType_BIND   0
#define PacketType_DATA   1  
#define PacketType_SYNC   2  
#define payloadsize       5
#define FHSShopInterval   4
Adafruit_SSD1306 display(OLED_RESET);
WORD_ALIGNED_ATTR typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;
Packet_t packet;
// send rate
uint16_t sendcount;
uint16_t sendfreq;
// now time
uint32_t now;
// send data
uint8_t tx_data;
// bind status
bool inBindingMode;
// send bind packet count
uint8_t bindcount;
// FHSS hop count 
volatile uint8_t OtaNonce;
// current freq
uint32_t currentFreq;
// currunt channel
uint8_t currentchannel;
uint16_t count;
// transmit status
volatile bool busyTransmitting;

void SetRFLinkRate(uint8_t index) // Set speed of RF link
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
    // Binding always uses invertIQ

    uint32_t interval = ModParams->interval;
    hwTimer::updateInterval(interval);

    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, 0, ModParams->PayloadLength, 0
                 , uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);


    // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
    FHSSsetCurrIndex(0);
    OtaNonce = 0;

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
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
void tock() 
{
    // Serial.println(millis());
    if(count % 500 == 0)
    {
        count = 0;
        Serial.println(sendcount);
        sendcount = 0;
    }
    if(!inBindingMode)
    {
        packet.type = PacketType_DATA;
        packet.IntervalCount = OtaNonce;
        currentchannel = FHSSgetCurrIndex();
        packet.currentchannel = currentchannel;
        packet.payloadSize = 1;
        memcpy(packet.payload, &tx_data, packet.payloadSize);
        tx_data++;
        // send
        busyTransmitting = true;
        Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
        while(!inBindingMode && busyTransmitting){yield();}
    }
    count++;
}
// 处理发送完毕中断
void ICACHE_RAM_ATTR TXdoneCallback()
{
    sendcount++;
    busyTransmitting = false;
    if(inBindingMode == false)
    {
        OtaNonce++;
        if(OtaNonce % FHSShopInterval == 0)
        {
            OtaNonce = 0;
            currentFreq = FHSSgetNextFreq();
            Radio.SetFrequencyReg(currentFreq);
        }
    }
    else {bindcount++;}
}
// 初始化
void setup()
{
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
    hwTimer::init(nullptr, tock);
    hwTimer::resume();
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
    now = millis();
}
// 主循环
void loop()
{
    // Freq
    display.clearDisplay();  
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
    display.println(tx_data);  
    // Rate
    display.setCursor(0, 24);   
    display.println("Rate");     
    display.setCursor(30, 24);         
    display.println(sendfreq);
    display.display();

    if(inBindingMode)
    { 
        // load
        packet.type = PacketType_BIND;
        packet.IntervalCount = OtaNonce;
        currentchannel = FHSSgetCurrIndex();
        packet.currentchannel = currentchannel;
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

}


