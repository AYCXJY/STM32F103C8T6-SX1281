#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "targets.h"
#include "common.h"
#include "SX1280Driver.h"
#include "FHSS.h"
#include "hwTimer.h"


#define airRate RATE_LORA_150HZ


// oled setting
#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64
// packet type
#define PacketType_BIND   0
#define PacketType_DATA   1  
#define PacketType_SYNC   2  
#define payloadsize       5
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
// send data
uint8_t tx_data;
// bind status
bool inBindingMode;
// FHSS hop count 
volatile uint8_t OtaNonce;
// current freq
uint32_t currentFreq;
// currunt channel
uint8_t currentchannel;
// transmit status
volatile bool busyTransmitting;

void SetRFLinkRate(uint8_t index) 
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

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);


    currentchannel = 0;
    FHSSsetCurrIndex(currentchannel);
    OtaNonce = 0;

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
}

static void setupBindingFromConfig()
{
    UID[0] = (uint8_t)HAL_GetUIDw0();
    UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
    UID[2] = (uint8_t)HAL_GetUIDw1();
    UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
    UID[4] = (uint8_t)HAL_GetUIDw2();
    UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);

    Serial.print("UID ");
    for(int i = 0; i < sizeof(UID); i++)
        Serial.print(String(UID[i]) + " ");
    Serial.println();
}

void exitbindingmode(void)
{
    if(inBindingMode == true)
    {
        inBindingMode = false;  
        SetRFLinkRate(enumRatetoIndex(airRate));
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
    }
}

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

void handleButtonPress()
{
  enterbindingmode();
}

void sendData(void)
{
    // make data packet
    packet.type = PacketType_DATA;
    packet.IntervalCount = OtaNonce;
    currentchannel = FHSSgetCurrIndex();
    packet.currentchannel = currentchannel;
    packet.payloadSize = 1;
    memcpy(packet.payload, &tx_data, packet.payloadSize);
    tx_data++;
    // send data packet
    busyTransmitting = true;
    Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
    while(!inBindingMode && busyTransmitting){yield();}
}

void sendUID(void)
{
    // make UID packet
    packet.type = PacketType_BIND;
    packet.IntervalCount = OtaNonce;
    currentchannel = FHSSgetCurrIndex();
    packet.currentchannel = currentchannel;
    packet.payloadSize = sizeof(UID) - 2;
    memcpy(packet.payload, UID + 2, packet.payloadSize);
    // send packet 20 times
    for(int i = 0; i < 20; i++)
    {
        busyTransmitting = true;
        Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
        while(busyTransmitting){yield();}
    }
}

void OLEDdisplayDebugInfo()
{
    display.clearDisplay();  

    if(inBindingMode)
    {            
        display.setCursor(0, 0);            
        display.println("sending UID...");
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
           
    display.display();
}

void ICACHE_RAM_ATTR timerCallback()
{
        static uint16_t tockcount;

        if(!inBindingMode)
        {
            // 记录每秒发包数
            if(tockcount >= (1000000 / ExpressLRS_currAirRate_Modparams->interval))
            {
                tockcount = 0;
                sendfreq = sendcount;
                sendcount = 0;
            }
            sendData();
        }
        tockcount++;
}

void ICACHE_RAM_ATTR TXdoneCallback()
{
    OtaNonce++;
    if(OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0)
    {
        OtaNonce = 0;
        currentFreq = FHSSgetNextFreq();
        Radio.SetFrequencyReg(currentFreq);
    }
    sendcount++;
    busyTransmitting = false;
}

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
    SetRFLinkRate(enumRatetoIndex(airRate));
    // timer
    hwTimer::init(nullptr, timerCallback);
    hwTimer::resume();
}

void loop()
{
    if(inBindingMode)
    { 
        OLEDdisplayDebugInfo();
        sendUID();
        exitbindingmode();
    }

    OLEDdisplayDebugInfo();
}


