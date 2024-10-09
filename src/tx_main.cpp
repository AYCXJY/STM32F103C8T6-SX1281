#include <Arduino.h>
#include <time.h>

#include "targets.h"
#include "common.h"

#include "SX1280Driver.h"
#include "FHSS.h"
// OLED
#include <Adafruit_SSD1306.h>
#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(OLED_RESET);
// Packet
#define PacketType_BIND   0
#define PacketType_DATA   1  
#define payloadsize       5
volatile bool busyTransmitting;
WORD_ALIGNED_ATTR typedef struct __attribute__((packed)) {
    uint8_t   type:2,
              IntervalCount:6;
    uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[payloadsize];
} Packet_t;
Packet_t packet;
uint8_t tx_data;
// BIND
bool inBindingMode;
static uint8_t BindingSendCount;
uint32_t BindingTime;
// FHSS
uint8_t FHSShopInterval = 4;    
uint8_t IntervalCount = 0;
uint32_t currentFreq = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
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
  busyTransmitting = false;
  digitalToggle(PC13);
}

void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, 0);
  ExpressLRS_currAirRate_Modparams = ModParams;
}

static void setupBindingFromConfig()
{
  UID[0] = (uint8_t)HAL_GetUIDw0();
  UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
  UID[2] = (uint8_t)HAL_GetUIDw1();
  UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
  UID[4] = (uint8_t)HAL_GetUIDw2();
  UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);
 
  // print UID 
  // Serial.print("UID = ");
  // for(int i = 0; i < 6; i++)  
  // {
  //   Serial.print(UID[i]);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
}

void enterbindingmode(void)
{
    SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
    FHSSsetCurrIndex(0);
    currentFreq = FHSSgetInitialFreq();
    Radio.SetFrequencyReg(currentFreq);
    inBindingMode = true;
}

void exitbindingmode(void)
{
  SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
  inBindingMode = false;  
}

void handleButtonPress() 
{
  if(inBindingMode == false)
  {
    enterbindingmode();
    BindingTime = millis();
  }   
}

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
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); 
  }
  display.setTextSize(1);            
  display.setTextColor(WHITE);        
  display.clearDisplay(); 
  display.display();
  // SX1280
  setupBindingFromConfig();
  pinMode(GPIO_PIN_TX_EN, OUTPUT);
  pinMode(GPIO_PIN_RX_EN, OUTPUT);
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.TXdoneCallback = &TXdoneCallback;
  currentFreq = FHSSgetInitialFreq(); 
  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
}

void loop()
{
    if(inBindingMode)
    {
        packet.type = PacketType_BIND;
        packet.IntervalCount = 0;
        packet.currentchannel = FHSSgetCurrIndex();
        packet.payloadSize = sizeof(UID) - 2;
        memcpy(packet.payload, UID + 2, packet.payloadSize);

        display.clearDisplay();          
        display.setCursor(0, 0);            
        display.println("sending UID...");
        // UID          
        display.setCursor(0, 12);           
        display.println("UID");         
        display.setCursor(24, 12);            
        display.println(UID[2]);
        display.setCursor(48, 12);            
        display.println(UID[3]);
        display.setCursor(72, 12);            
        display.println(UID[4]);
        display.setCursor(96, 12);            
        display.println(UID[5]);
        // FHSS Hop Freq
        display.setCursor(0, 24);           
        display.println("Freq");    
        display.setCursor(30, 24);           
        display.println(currentFreq);  
        display.setCursor(94, 24);           
        display.println(packet.currentchannel);  
        display.display();

        while(millis() - BindingTime < 1000)
        {
            busyTransmitting = true;
            Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);

            uint8_t output[8];
            memcpy(output, &packet, 8);
            for(int i = 0; i < sizeof(packet); i++)
            {
                Serial.print(output[i]);
                Serial.print(" ");
            }
            Serial.println(" ");

            while(busyTransmitting)
            {
                yield();
            }
        }
        exitbindingmode();
  }
  else
  {
    packet.type = PacketType_DATA;
    packet.IntervalCount = IntervalCount;
    packet.currentchannel = FHSSgetCurrIndex();
    packet.payloadSize = 1;
    memcpy(packet.payload, &tx_data, packet.payloadSize);
    Radio.TXnb((uint8_t*)&packet, 8, SX12XX_Radio_1);
    tx_data++;

    // print packet
    uint8_t output[8];
    memcpy(output, &packet, 8);
    for(int i = 0; i < sizeof(packet); i++)
    {
        Serial.print(output[i]);
        Serial.print(" ");
    }
    Serial.println(" ");

    display.clearDisplay();  
    // UID          
    display.setCursor(0, 0);           
    display.println("UID");         
    display.setCursor(24, 0);            
    display.println(UID[2]);
    display.setCursor(48, 0);            
    display.println(UID[3]);
    display.setCursor(72, 0);            
    display.println(UID[4]);
    display.setCursor(96, 0);            
    display.println(UID[5]);
    // FHSS Hop Freq
    display.setCursor(0, 12);           
    display.println("Freq");    
    display.setCursor(30, 12);           
    display.println(currentFreq);  
    display.setCursor(94, 12);           
    display.println(packet.currentchannel);  
    display.setCursor(118, 12);           
    display.println(packet.IntervalCount);  
    // Data 
    display.setCursor(0, 24);           
    display.println("Data");  
    display.setCursor(30, 24);           
    display.println(tx_data);  
    display.display();
   

    // // 调试信息
    // Radio.GetStatus(SX12XX_Radio_1);
    // Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
    // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
    // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
    // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));
    // Serial.println("Current Index = " + String(FHSSgetCurrIndex()));

    delay(50);
    // yield();
  }
}


