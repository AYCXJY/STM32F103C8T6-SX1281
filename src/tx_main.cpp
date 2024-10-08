#include <Arduino.h>
#include <time.h>

#include "targets.h"
#include "common.h"

#include "SX1280Driver.h"
#include "FHSS.h"

#include <Adafruit_SSD1306.h>

#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64

Adafruit_SSD1306 display(OLED_RESET);

#define PacketType_BIND   0
#define PacketType_DATA   1  

typedef struct __attribute__((packed)) {
    uint8_t   type;
    // uint8_t   IntervalCount;
    // uint8_t   currentchannel;
    uint8_t   payloadSize;
    uint8_t   payload[6];
} Packet_t;

Packet_t packet;

#define BindingSpamAmount 25
bool inBindingMode;
static uint8_t BindingSendCount;
uint32_t BindingTime;
uint8_t sendUIDcount;

// uint8_t FHSShopInterval = 4;    
// uint8_t IntervalCount = 0;
// uint32_t currentFreq = 0;

void ICACHE_RAM_ATTR TXdoneCallback()
{
  // IntervalCount++;
  // if(IntervalCount % FHSShopInterval == 0)
  // {
  //   IntervalCount = 0;
  //   currentFreq = FHSSgetNextFreq();
  //   Radio.SetFrequencyReg(currentFreq);
  // }
  digitalToggle(PC13);
}

void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));
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

  Serial.print("UID = ");
  for(int i = 0; i < 6; i++)  
  {
    Serial.print(UID[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}

void handleButtonPress() 
{
  if(!inBindingMode)
  {
    inBindingMode = true;
    BindingTime = millis();
  }   
  digitalToggle(PC13);
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
  Radio.currFreq = FHSSgetInitialFreq(); 
  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
}

void loop()
{
  if(inBindingMode)
  {
    SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
    sendUIDcount = 0;
    packet.type = PacketType_BIND;
    packet.payloadSize = 6;
    
    for(int i = 0; i < packet.payloadSize; i++)
    {
      packet.payload[i] = UID[i];
    }
    while(millis() - BindingTime < 5000)
    {
      Radio.TXnb((uint8_t*)&packet, sizeof(packet), SX12XX_Radio_1);
      sendUIDcount++;
      delay(500);
      display.clearDisplay();             
      display.setCursor(0, 0);            
      display.println("sending UID...");
      display.setCursor(0, 8);            
      display.println(sendUIDcount);
      display.display();
    }
    inBindingMode = false;  
    SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
  }
  else
  {
    // Radio.TXnb(data, sizeof(data), SX12XX_Radio_1);

    // 调试信息
    // Radio.GetStatus(SX12XX_Radio_1);
    // Serial.println(Radio.GetRssiInst(SX12XX_Radio_1));
    // Serial.println("Initial Freq = " + String(FHSSgetInitialFreq()));
    // Serial.println("Sequense Count = " + String(FHSSgetSequenceCount()));
    // Serial.println("Channel Count = " + String(FHSSgetChannelCount()));
    // Serial.println("Current Index = " + String(FHSSgetCurrIndex()));
    display.clearDisplay();            
    display.setCursor(0, 0);           
    display.println("UID");         
    display.setCursor(0, 8);            
    display.println(packet.payload[0]);
    display.setCursor(24, 8);            
    display.println(packet.payload[1]);
    display.setCursor(48, 8);            
    display.println(packet.payload[2]);
    display.setCursor(0, 16);            
    display.println(packet.payload[3]);
    display.setCursor(24, 16);            
    display.println(packet.payload[4]);
    display.setCursor(48, 16);            
    display.println(packet.payload[5]);
    display.display();
    delay(1000);
    yield();
  }
}


