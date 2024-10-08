#include <Arduino.h>

#include "targets.h"
#include "common.h"

#include "SX1280Driver.h"
#include "FHSS.h"

#include <Adafruit_SSD1306.h>

#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64

Adafruit_SSD1306 display(OLED_RESET);

bool inBindingMode;
#define PacketType_BIND   0
#define PacketType_DATA   1  
// Bound is any of the last 4 bytes nonzero (unbound is all zeroes)
#define UID_IS_BOUND(uid) (uid[2] != 0 || uid[3] != 0 || uid[4] != 0 || uid[5] != 0)

void SetRFLinkRate(uint8_t index){
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));
  ExpressLRS_currAirRate_Modparams = ModParams;
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const status)
{
  if(inBindingMode && Radio.RXdataBuffer[0] == PacketType_BIND)
  {
    uint8_t payloadsize = Radio.RXdataBuffer[1];
    for(int i = 0; i < payloadsize ; i++)
    {
      UID[i+2] = Radio.RXdataBuffer[i+2];
    }
    inBindingMode = false;

    // 打印 RXdataBuffer
    for (int i = 0; i < 16; i++)
    {
      Serial.print(Radio.RXdataBuffer[i]);
      Serial.print(" ");
    }
    Serial.println(" ");

    Serial.print("UID = ");
    for(int i = 0; i < 6; i++)  
    {
      Serial.print(UID[i]);
      Serial.print(" ");
    }
    Serial.println(" ");  
    // SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
  // }
  }
    return true;
}

void setup()
{
  // UART
  Serial.begin(115200);
  // LED
  pinMode(PC13, OUTPUT);
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
  pinMode(GPIO_PIN_TX_EN, OUTPUT);
  pinMode(GPIO_PIN_RX_EN, OUTPUT);
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.RXdoneCallback = &RXdoneCallback;
  Radio.currFreq = FHSSgetInitialFreq(); 
  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
  SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
  // 开启连续接收模式
  Radio.RXnb(SX1280_MODE_RX_CONT);
}

void enterBindingMode()
{
  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
  Radio.RXnb(SX1280_MODE_RX_CONT);
}

void loop()
{
  if(inBindingMode == false && !UID_IS_BOUND(UID))
  {
    inBindingMode = true;
    enterBindingMode();
    display.clearDisplay();             
    display.setCursor(0, 0);            
    display.println("receiving UID...");
    display.display();
  }
  display.clearDisplay();            
  display.setCursor(0, 0);           
  display.println("UID");
        
  display.setCursor(0, 8);            
  display.println(UID[0]);

  display.setCursor(18, 8);            
  display.println(UID[1]);

  display.setCursor(40, 8);            
  display.println(UID[2]);

  display.setCursor(56, 8);            
  display.println(UID[3]);

  display.setCursor(72, 8);            
  display.println(UID[4]);

  display.setCursor(88, 8);            
  display.println(UID[5]);
  display.display();

  digitalToggle(PC13);
  delay(200);
  yield();
}
