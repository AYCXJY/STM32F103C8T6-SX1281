#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#include "targets.h"
#include "common.h"
// #include "config.h"
#include "CRSF.h"
#include "FHSS.h"
// #include "helpers.h"
#include "hwTimer.h"
// #include "logging.h"
// #include "LBT.h"
#include "LQCALC.h"
#include "OTA.h"
// #include "POWERMGNT.h"
// #include "deferred.h"

// #include "CRSFHandset.h"
// #include "dynpower.h"
// #include "lua.h"
#include "msp.h"
#include "msptypes.h"
#include "telemetry_protocol.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"


#define airRate RATE_LORA_50HZ

#define syncSpamAmount 3
#define syncSpamAmountAfterRateChange 10
volatile uint8_t syncSpamCounter = 0;
volatile uint8_t syncSpamCounterAfterRateChange = 0;
uint32_t rfModeLastChangedMS = 0;
uint32_t SyncPacketLastSent = 0;

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
static uint32_t now;

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
    if(InBindingMode == true)
    {
        InBindingMode = false;  
        SetRFLinkRate(enumRatetoIndex(airRate));
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
    }
}

void enterbindingmode(void)
{
    if(InBindingMode == false)
    {
        InBindingMode = true;
        SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
    }   
}

void ICACHE_RAM_ATTR HandleFHSS()
{
    uint8_t modresult = (OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    // If the next packet should be on the next FHSS frequency, do the hop
    if (!InBindingMode && modresult == 0)
    {
        Radio.SetFrequencyReg(FHSSgetNextFreq());
    }
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

    if(InBindingMode)
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
    if (!InBindingMode)
        OtaNonce++;

    SendRCdataToRF();
}

void ICACHE_RAM_ATTR TXdoneISR()
{
    HandleFHSS();
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
    attachInterrupt(digitalPinToInterrupt(PB1), enterbindingmode, FALLING);  
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
    Radio.TXdoneCallback = &TXdoneISR;
    currentFreq = FHSSgetInitialFreq(); 
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    SetRFLinkRate(enumRatetoIndex(airRate));
    // timer
    hwTimer::init(nullptr, timerCallback);
    hwTimer::resume();
}

void loop()
{
    if(InBindingMode)
    {
        OLEDdisplayDebugInfo();
        sendUID();
        exitbindingmode();
    }
    OLEDdisplayDebugInfo();
}


