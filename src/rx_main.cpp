#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "TimerInterrupt_Generic.h"

#include "targets.h"
#include "common.h"
#include "SX1280Driver.h"
#include "FHSS.h"
#include "hwTimer.h"

#include "elrs_eeprom.h"
#include "PFD.h"
#include "LowPassFilter.h"

#define airRate RATE_LORA_500HZ

#define TIMER_INTERVAL_MS 1000000
STM32Timer ITimer(TIM2);

#define OLED_RESET        4 
#define SCREEN_WIDTH      128 
#define SCREEN_HEIGHT     64

#define PacketType_BIND   0
#define PacketType_DATA   1  
#define PacketType_SYNC   2  
#define payloadsize       5

#define PACKET_TO_TOCK_SLACK 200

Adafruit_SSD1306 display(OLED_RESET);

ELRS_EEPROM eeprom;

PFD PFDloop;

WORD_ALIGNED_ATTR typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;
Packet_t packet;

uint16_t receivecount;
uint16_t receivefreq;

uint8_t rx_data;

bool inBindingMode;
volatile uint8_t OtaNonce;
uint32_t currentFreq;
uint8_t currentchannel;
uint8_t uplinkLQ;
LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);
int32_t PfdPrevRawOffset;
RXtimerState_e RXtimerState;

uint32_t ticktime;
uint32_t tocktime;
uint32_t rxdonetime;

void TimerHandler() 
{
    digitalToggle(PC13);
    receivefreq = receivecount;
    receivecount = 0;
}

void SetRFLinkRate(uint8_t index, bool bindMode)
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
}

void exitbindingmode(void)
{
    if(inBindingMode == true)
    {
        hwTimer::resume();
        inBindingMode = false;
        SetRFLinkRate(enumRatetoIndex(airRate), false);
        currentchannel = 0;
        FHSSsetCurrIndex(currentchannel);
        currentFreq = FHSSgetInitialFreq();
        Radio.SetFrequencyReg(currentFreq);
        Radio.RXnb(SX1280_MODE_RX_CONT);

        FHSSrandomiseFHSSsequence(uidMacSeedGet());
    }
}

void enterbindingmode(void)
{
    if(inBindingMode == false)
    {
        hwTimer::stop();
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
    Serial.println("Button Pressed");
    enterbindingmode();
}

void OLEDdisplayDebugInfo()
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
    // RSSI
    display.setCursor(54, 24);     
    display.println("RSSI");     
    display.setCursor(84, 24);           
    display.println(Radio.GetRssiInst(SX12XX_Radio_1)); 

    display.display();
}

void ICACHE_RAM_ATTR updatePhaseLock()
{
    if (PFDloop.hasResult())
    {
        int32_t RawOffset = PFDloop.calcResult();
        int32_t Offset = LPF_Offset.update(RawOffset);
        int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
        PfdPrevRawOffset = RawOffset;

        if (Offset > 0)
        {
            hwTimer::incFreqOffset();
        }
        else if (Offset < 0)
        {
            hwTimer::decFreqOffset();
        }
        if(tocktime - rxdonetime > 500)
            hwTimer::phaseShift(Offset >> 2);
        else
            hwTimer::phaseShift(RawOffset >> 1);

        Serial.println("Offset " + String(Offset) + "  RawOffset " + String(RawOffset) + "  OffsetDx " + String(OffsetDx));
        Serial.println("FreqOffset " + String(hwTimer::getFreqOffset()));

        UNUSED(OffsetDx); // complier warning if no debug
    }

    PFDloop.reset();
}

void ICACHE_RAM_ATTR timerCallbacktick()
{
    updatePhaseLock();
    OtaNonce++;

    // Serial.print("             tick interval " + String(micros() - ticktime));
    ticktime = micros();
    // Serial.println("                               tick " + String(ticktime - tocktime));
}

void ICACHE_RAM_ATTR timerCallbacktock()
{
    // Serial.println("tock interval " + String(micros() - tocktime));
    tocktime = micros();
    Serial.println("tock " + String(tocktime - rxdonetime));
    PFDloop.intEvent(tocktime); // our internal osc just fired

}

bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status)
{
    // Serial.println("rxdone interval " + String(micros() - rxdonetime));
    rxdonetime = micros();
    // Serial.println("rxdone " + String(rxdonetime));
    uint32_t const beginProcessing = micros();
    PFDloop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);

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
      if((OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0)
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
    Radio.RXdoneCallback = &RXdoneISR;
    // Radio.TXdoneCallback = &TXdoneISR;
    currentFreq = FHSSgetInitialFreq(); 
    Radio.currFreq = currentFreq;
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    SetRFLinkRate(enumRatetoIndex(airRate), false);
    // 开启连续接收模式
    Radio.RXnb(SX1280_MODE_RX_CONT);
    // timer
    hwTimer::init(timerCallbacktick, timerCallbacktock);
    hwTimer::resume();

    ITimer.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);

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
    // static uint32_t now = millis();
    // if(millis() - now >= 1000)
    // {
    //     now = millis();
    //     receivefreq = receivecount;
    //     receivecount = 0;
    // }

    OLEDdisplayDebugInfo();
}
