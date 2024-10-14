#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "targets.h"
#include "common.h"
#include "SX1280Driver.h"
#include "FHSS.h"
#include "hwTimer.h"
#include "elrs_eeprom.h"
#include "PFD.h"

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

#define UID_IS_BOUND(uid) (uid[2] != 255 || uid[3] != 255 || uid[4] != 255 || uid[5] != 255)

/***************** class *************************/
Adafruit_SSD1306 display(OLED_RESET);
ELRS_EEPROM eeprom;
PFD PFDloop;
/***************** global variable ***************/
// packet struct
WORD_ALIGNED_ATTR typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;
Packet_t packet;
// recieve rate
uint16_t receivecount;
uint16_t receivefreq;
// now time
uint32_t now;
// receive data
uint8_t rx_data;
// bind status
bool inBindingMode;
uint8_t ExpressLRS_nextAirRateIndex;
// FHSS hop count 
volatile uint8_t OtaNonce;
// current freq
uint32_t currentFreq;
// currunt channel
uint8_t currentchannel;
 

void SetRFLinkRate(uint8_t index, bool bindMode) // Set speed of RF link
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);

    uint32_t interval = ModParams->interval;
    hwTimer::updateInterval(interval);

    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, 0, ModParams->PayloadLength, 0
                 , uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));


    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshDn - RFperf->DynpowerSnrThreshUp);


    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index; // presumably we just handled this
}

void exitbindingmode(void)
{
    inBindingMode = false;
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ), false);
    currentchannel = 0;
    FHSSsetCurrIndex(currentchannel);
    currentFreq = FHSSgetInitialFreq();
    Radio.SetFrequencyReg(currentFreq);
    Radio.RXnb(SX1280_MODE_RX_CONT);

    FHSSrandomiseFHSSsequence(uidMacSeedGet());
}

void enterbindingmode(void)
{
    if(inBindingMode == false)
    {
        inBindingMode = true;
        SetRFLinkRate(enumRatetoIndex(RATE_BINDING), true);
        currentchannel = 0;
        FHSSsetCurrIndex(currentchannel);
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
        Radio.RXnb(SX1280_MODE_RX_CONT);
    }
}

void handleButtonPress() 
{
  enterbindingmode();
}

void tick() 
{
    // Serial.println("tick");
    // Serial.println(millis());
}

void tock() 
{
    static uint16_t count = 0;
    count++;
    if(count % 500 == 0)
    {
        receivefreq = receivecount;
        receivecount = 0;
        count = 0;
        // Serial.println(receivecount);
    }
}

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
      OtaNonce = PktPtr->IntervalCount;
      if(OtaNonce == FHSShopInterval - 1)
      {
        currentFreq = FHSSgetNextFreq();
        Radio.SetFrequencyReg(currentFreq);
      }
    }
    return true;
}

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
    FHSSrandomiseFHSSsequence(uidMacSeedGet());
    pinMode(GPIO_PIN_TX_EN, OUTPUT);
    pinMode(GPIO_PIN_RX_EN, OUTPUT);
    Radio.RXdoneCallback = &RXdoneCallback;
    currentFreq = FHSSgetInitialFreq(); 
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ), false);
    // 开启连续接收模式
    Radio.RXnb(SX1280_MODE_RX_CONT);
    // timer
    hwTimer::init(tick, tock);
    hwTimer::resume();
    // eeprom
    eeprom.Begin();
    eeprom.Get(0, UID);
    if(!UID_IS_BOUND(UID))
    {
        enterbindingmode();
    }
}

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
