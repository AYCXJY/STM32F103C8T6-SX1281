#include <Arduino.h>
#include "targets.h"
#include "SX1280Driver.h"
#include "common.h"
#include "FHSS.h"

// 定义无线电实例
SX1280Driver Radio;

// 定义发送数据
uint8_t testdata[] = "HELLO!";

// 发送完成处理函数
void ICACHE_RAM_ATTR TXdoneCallback()
{
    // 发送完数据翻转LED
    digitalToggle(PC13);
    // 输出信息到上位机串口
    Serial.println("TXdoneCallback");
}

// 接受完成处理函数
bool ICACHE_RAM_ATTR RXdoneCallback(SX12xxDriverCommon::rx_status const /*status*/)
{
    Serial.println("RXdoneCallback");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(Radio.RXdataBuffer[i], HEX);
        Serial.print(",");
    }
    Serial.println("");
    //Radio.RXnb();
    return true;
}

// 初始化函数
void setup()
{
    // 串口初始化
    Serial.begin(115200);
    Serial.println("Begin SX1280 testing...");

    // 初始化LED
    pinMode(PC13, OUTPUT);

    // 初始化无线电
    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    //Radio.Config(SX1280_LORA_BW_0800, SX1280_LORA_SF6, SX1280_LORA_CR_4_7, 2420000000, SX1280_PREAMBLE_LENGTH_32_BITS);
    Radio.TXdoneCallback = &TXdoneCallback;
    Radio.RXdoneCallback = &RXdoneCallback;
    Radio.SetFrequencyReg(FHSSgetInitialFreq());
    //Radio.RXnb();
    // Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_1);
}

// 主循环
void loop()
{
    // Serial.println("about to TX");
    Radio.TXnb(testdata, sizeof(testdata), SX12XX_Radio_1);
    delay(1000);

    // Serial.println("about to RX");
    //Radio.RXnb();
    // delay(100);
    //delay(random(50,200));
    //delay(100);
    //Radio.GetStatus();

    yield();
}