#include <Arduino.h>
#include <time.h>

#include "targets.h"
#include "common.h"

#include "elrs_eeprom.h"

#include "SX1280Driver.h"
#include "FHSS.h"

#include <Adafruit_SSD1306.h>

#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64

Adafruit_SSD1306 display(OLED_RESET);
ELRS_EEPROM eeprom;

#define PacketType_BIND   0
#define PacketType_DATA   1  

bool inBindingMode;
#define UID_IS_BOUND(uid) (uid[2] != 255 || uid[3] != 255 || uid[4] != 255 || uid[5] != 255)

void SetRFLinkRate(uint8_t index)
{
  expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(), 
              ModParams->PreambleLen, false, ModParams->PayloadLength, ModParams->interval, 
              uidMacSeedGet(), 0, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));
  ExpressLRS_currAirRate_Modparams = ModParams;
}

void enterbindingmode(void)
{
  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
  Radio.RXnb(SX1280_MODE_RX_CONT);
  inBindingMode = true;
}

void exitbindingmode(void)
{
  SetRFLinkRate(enumRatetoIndex(RATE_LORA_500HZ));
  Radio.RXnb(SX1280_MODE_RX_CONT);
  inBindingMode = false;
}

void handleButtonPress() 
{
  enterbindingmode();
}

bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const status)
{
  if(inBindingMode && Radio.RXdataBuffer[0] == PacketType_BIND)
  {
    // 
    uint8_t payloadsize = Radio.RXdataBuffer[1];
    for(int i = 0; i < payloadsize ; i++){
      UID[i] = Radio.RXdataBuffer[i+2];
    }
    // put UID to eeprom
    eeprom.Put(0, UID);
    eeprom.Commit();
    exitbindingmode();

    // // 打印 RXdataBuffer
    // for (int i = 0; i < 16; i++)
    // {
    //   Serial.print(Radio.RXdataBuffer[i]);
    //   Serial.print(" ");
    // }
    // Serial.println(" ");
    // Serial.print("UID = ");
    // for(int i = 0; i < 6; i++)  
    // {
    //   Serial.print(UID[i]);
    //   Serial.print(" ");
    // }
    // Serial.println(" ");  
  }
    return true;
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
  pinMode(GPIO_PIN_TX_EN, OUTPUT);
  pinMode(GPIO_PIN_RX_EN, OUTPUT);
  FHSSrandomiseFHSSsequence(uidMacSeedGet());
  Radio.RXdoneCallback = &RXdoneCallback;
  Radio.currFreq = FHSSgetInitialFreq(); 
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
}

void loop()
{
  if(inBindingMode)
  {
    display.clearDisplay();             
    display.setCursor(0, 0);            
    display.println("receiving UID...");
    display.display();
  }
  else
  {
    display.clearDisplay();    
    // print UID
    display.setCursor(0, 0);           
    display.println("UID");         
    display.setCursor(0, 8);            
    display.println(UID[0]);
    display.setCursor(24, 8);            
    display.println(UID[1]);
    display.setCursor(48, 8);            
    display.println(UID[2]);
    display.setCursor(0, 16);            
    display.println(UID[3]);
    display.setCursor(24, 16);            
    display.println(UID[4]);
    display.setCursor(48, 16);            
    display.println(UID[5]);
    // print RXBuff
    display.setCursor(0, 24);            
    display.println(&Radio.RXdataBuffer);

    display.display();
  }
  yield();
}
