// 日期：2024年10月23日

// 完成项：
// 默认绑定与换绑（使用EEPROM存储UID）（使用OTA封装发送绑定包）
// FHSS跳频通信
// 可选通信速率（Lora MAX -> 500HZ）
// 同步和断线重连

// 下阶段：实现一个无线双向透传，不需要确认和重传。

// 需求项：发射机和接收机都需要启用半双工模式，依据TLM回传逻辑收发包，使用PC串口接收和发送数据。
// 需求细分：第一步：实现从PC串口助手读取数据后回传；(完成)
//          第二步：实现单向透传；
//          第三步：启用TLM，实现双向透传；

// 可选功能项：
// 使用Stubborn实现可靠数据传输
// 添加遥测数据回传
// 通道数据填充与解析
// 链路质量

// 优化项：
// 使用Radio.Rxnb()单次传输降低功耗
// 动态功率降低功耗
// logging调试库的使用



/* ELRS include */

#include "rxtx_common.h"
#include "LowPassFilter.h"

#include "crc.h"
#include "telemetry_protocol.h"
#include "telemetry.h"
#include "stubborn_sender.h"
#include "stubborn_receiver.h"

// #include "lua.h"
#include "msp.h"
#include "msptypes.h"
#include "PFD.h"
// #include "options.h"
// #include "dynpower.h"
#include "MeanAccumulator.h"
// #include "freqTable.h"

// #include "rx-serial/SerialIO.h"
// #include "rx-serial/SerialNOOP.h"
// #include "rx-serial/SerialCRSF.h"
// #include "rx-serial/SerialSBUS.h"
// #include "rx-serial/SerialSUMD.h"
// #include "rx-serial/SerialAirPort.h"
// #include "rx-serial/SerialHoTT_TLM.h"
// #include "rx-serial/SerialMavlink.h"
// #include "rx-serial/SerialTramp.h"
// #include "rx-serial/SerialSmartAudio.h"
//
// #include "rx-serial/devSerialIO.h"
// #include "devLED.h"
// #include "devLUA.h"
// #include "devWIFI.h"
// #include "devButton.h"
// #include "devServoOutput.h"
// #include "devVTXSPI.h"
// #include "devAnalogVbat.h"
// #include "devSerialUpdate.h"
// #include "devBaro.h"
// #include "devMSPVTX.h"
// #include "devThermal.h"

/* User include */

#include <Adafruit_SSD1306.h>
#include "TimerInterrupt_Generic.h"
#include <algorithm> 
/* ELRS variable */

StubbornSender TelemetrySender;
static uint8_t telemetryBurstCount;
static uint8_t telemetryBurstMax;

StubbornReceiver MspReceiver;
uint8_t MspData[ELRS_MSP_BUFFER];

// Buffer for current stubbon sender packet (mavlink only)
uint8_t mavlinkSSBuffer[CRSF_MAX_PACKET_LEN]; 

static bool tlmSent = false;
static uint8_t NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
static bool telemBurstValid;

#define PACKET_TO_TOCK_SLACK 200 

uint32_t LastValidPacket = 0;           //Time the last valid packet was recv
uint32_t LastSyncPacket = 0;            //Time the last valid packet was recv

bool BindingModeRequest = false;
static uint32_t BindingRateChangeTime;
#define BindingRateChangeCyclePeriod 125

// 存储地址 0x0801FC00
ELRS_EEPROM eeprom;
PFD PFDloop;

LPF LPF_Offset(2);
LPF LPF_OffsetDx(4);

LQCALC<100> LQCalc;
LQCALC<100> LQCalcDVDA;
uint8_t uplinkLQ;
LPF LPF_UplinkRSSI0(5);  // track rssi per antenna
LPF LPF_UplinkRSSI1(5);
MeanAccumulator<int32_t, int8_t, -16> SnrMean;

static uint8_t scanIndex;
uint8_t ExpressLRS_nextAirRateIndex;
int8_t SwitchModePending;

int32_t PfdPrevRawOffset;
RXtimerState_e RXtimerState;
uint32_t GotConnectionMillis = 0;
// minimum time before we can consider a connection to be 'good'
const uint32_t ConsiderConnGoodMillis = 1000; 
bool doStartTimer = false;

bool didFHSS = false;
bool alreadyFHSS = false;
bool alreadyTLMresp = false;

/// Variables for Sync Behaviour ////
uint32_t cycleInterval; // in ms
uint32_t RFmodeLastCycled = 0;
#define RFmodeCycleMultiplierSlow 10
uint8_t RFmodeCycleMultiplier;
bool LockRFmode = false;

/* User variable */

#define OLED_RESET     4 
#define SCREEN_WIDTH   128 
#define SCREEN_HEIGHT  64
Adafruit_SSD1306 display(OLED_RESET);

#define airRate RATE_LORA_500HZ

uint16_t receivecount;
uint16_t receivefreq;

uint8_t CRCvalue;

#define TIMER_INTERVAL_MS 1000000
STM32Timer ITimer(TIM2);

bool UIDIsModified = false;

uint32_t ticktime;
uint32_t tocktime;
uint32_t RxISRtime;
uint32_t slack;

FIFO<AP_MAX_BUF_LEN> apInputBuffer;
FIFO<AP_MAX_BUF_LEN> apOutputBuffer;

/* ELRS Function*/

static uint8_t minLqForChaos() // ELRS移植，注释源码另起修改
{
    // Determine the most number of CRC-passing packets we could receive on
    // a single channel out of 100 packets that fill the LQcalc span.
    // The LQ must be GREATER THAN this value, not >=
    // The amount of time we coexist on the same channel is
    // 100 divided by the total number of packets in a FHSS loop (rounded up)
    // and there would be 4x packets received each time it passes by so
    // FHSShopInterval * ceil(100 / FHSShopInterval * numfhss) or
    // FHSShopInterval * trunc((100 + (FHSShopInterval * numfhss) - 1) / (FHSShopInterval * numfhss))
    // With a interval of 4 this works out to: 2.4=4, FCC915=4, AU915=8, EU868=8, EU/AU433=36
    const uint32_t numfhss = FHSSgetChannelCount();
    const uint8_t interval = ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    return interval * ((interval * numfhss + 99) / (interval * numfhss));
}

void SetRFLinkRate(uint8_t index, bool bindMode) // ELRS移植，注释源码另起修改
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
    // Binding always uses invertIQ
    bool invertIQ = bindMode || (UID[5] & 0x01);

    uint32_t interval = ModParams->interval;
#if defined(DEBUG_FREQ_CORRECTION) && defined(RADIO_SX128X)
    interval = interval * 12 / 10; // increase the packet interval by 20% to allow adding packet header
#endif

    hwTimer::updateInterval(interval);

    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, 0
#if defined(RADIO_SX128X)
                 , uidMacSeedGet(), OtaCrcInitializer, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC)
#endif
#if defined(RADIO_LR1121)
               , ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 || ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4, (uint8_t)UID[5], (uint8_t)UID[4]
#endif
                 );

#if defined(RADIO_LR1121)
    if (FHSSuseDualBand)
    {
        Radio.Config(ModParams->bw2, ModParams->sf2, ModParams->cr2, FHSSgetInitialGeminiFreq(),
                    ModParams->PreambleLen2, invertIQ, ModParams->PayloadLength, 0,
                    ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 || ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4,
                    (uint8_t)UID[5], (uint8_t)UID[4], SX12XX_Radio_2);
    }
#endif

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshDn - RFperf->DynpowerSnrThreshUp);

    // checkGeminiMode();
    // if (geminiMode)
    // {
    //     Radio.SetFrequencyReg(FHSSgetInitialGeminiFreq(), SX12XX_Radio_2);
    // }

    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
    // MspReceiver.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
    // TelemetrySender.setMaxPackageIndex(OtaIsFullRes ? ELRS8_TELEMETRY_MAX_PACKAGES : ELRS4_TELEMETRY_MAX_PACKAGES);

    // // Wait for (11/10) 110% of time it takes to cycle through all freqs in FHSS table (in ms)
    // cycleInterval = ((uint32_t)11U * FHSSgetChannelCount() * ModParams->FHSShopInterval * interval) / (10U * 1000U);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    ExpressLRS_nextAirRateIndex = index; // presumably we just handled this
    telemBurstValid = false;
}

bool ICACHE_RAM_ATTR HandleFHSS() // ELRS移植，注释源码另起修改
{
    uint8_t modresultFHSS = (OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    if ((ExpressLRS_currAirRate_Modparams->FHSShopInterval == 0) || alreadyFHSS == true || InBindingMode || (modresultFHSS != 0) || (connectionState == disconnected))
    {
        return false;
    }

    alreadyFHSS = true;

    // if (geminiMode)
    // {
    //     if ((((OtaNonce + 1)/ExpressLRS_currAirRate_Modparams->FHSShopInterval) % 2 == 0) || FHSSuseDualBand) // When in DualBand do not switch between radios.  The OTA modulation paramters and HighFreq/LowFreq Tx amps are set during Config.
    //     {
    //         Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1);
    //         Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2);
    //     }
    //     else
    //     {
    //         // Write radio1 first. This optimises the SPI traffic order.
    //         uint32_t freqRadio2 = FHSSgetNextFreq();
    //         Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_1);
    //         Radio.SetFrequencyReg(freqRadio2, SX12XX_Radio_2);
    //     }
    // }
    // else
    {
        Radio.SetFrequencyReg(FHSSgetNextFreq());
    }

#if defined(RADIO_SX127X)
    // SX127x radio has to reset receive mode after hopping
    uint8_t modresultTLM = (OtaNonce + 1) % ExpressLRS_currTlmDenom;
    if (modresultTLM != 0 || ExpressLRS_currTlmDenom == 1) // if we are about to send a tlm response don't bother going back to rx
    {
        Radio.RXnb();
    }
#endif
#if defined(Regulatory_Domain_EU_CE_2400)
    SetClearChannelAssessmentTime();
#endif
    return true;
}

int32_t ICACHE_RAM_ATTR HandleFreqCorr(bool value) // ELRS移植，注释源码另起修改
{
    int32_t tempFC = FreqCorrection;
    if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_2)
    {
        tempFC = FreqCorrection_2;
    }

    if (value)
    {
        if (tempFC > FreqCorrectionMin)
        {
            tempFC--; // FREQ_STEP units
            if (tempFC == FreqCorrectionMin)
            {
                Serial.println("Max -FreqCorrection reached!");
                // DBGLN("Max -FreqCorrection reached!");
            }
        }
    }
    else
    {
        if (tempFC < FreqCorrectionMax)
        {
            tempFC++; // FREQ_STEP units
            if (tempFC == FreqCorrectionMax)
            {
                Serial.println("Max +FreqCorrection reached!");
                // DBGLN("Max +FreqCorrection reached!");
            }
        }
    }

    if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1)
    {
        FreqCorrection = tempFC;
    }
    else
    {
        FreqCorrection_2 = tempFC;
    }

    return tempFC;
}

void ICACHE_RAM_ATTR updatePhaseLock() // ELRS移植，注释源码另起修改
{
    if (/*connectionState != disconnected && */PFDloop.hasResult() && PFDloop.calcResult() < 1500)
    {
        int32_t RawOffset = PFDloop.calcResult();
        int32_t Offset = LPF_Offset.update(RawOffset);
        int32_t OffsetDx = LPF_OffsetDx.update(RawOffset - PfdPrevRawOffset);
        PfdPrevRawOffset = RawOffset;

        // if (RXtimerState == tim_locked)
        {
            // limit rate of freq offset adjustment, use slot 1
            // because telemetry can fall on slot 1 and will
            // never get here
            if (OtaNonce % 8 == 1)
            {
                if (Offset > 0)
                {
                    hwTimer::incFreqOffset();
                }
                else if (Offset < 0)
                {
                    hwTimer::decFreqOffset();
                }
            }
        }
        // if (slack > 500)
        // // if (connectionState != connected)
        // {
        //     hwTimer::phaseShift(RawOffset >> 1);
        // }
        // else
        {
            hwTimer::phaseShift(Offset >> 2);
        }

        // Serial.println("OFST " + String(Offset) + " ROFST " + String(RawOffset)
        //  + " OFSTD " + String(OffsetDx) + " HWTOFST " + String(hwTimer::getFreqOffset())/* + String(uplinkLQ)*/);
        // DBGVLN("%d:%d:%d:%d:%d", Offset, RawOffset, OffsetDx, hwTimer::getFreqOffset(), uplinkLQ);
        UNUSED(OffsetDx); // complier warning if no debug
    }
    PFDloop.reset();
}

void ICACHE_RAM_ATTR HWtimerCallbackTick() // ELRS移植，注释源码另起修改
{   // this is 180 out of phase with the other callback, occurs mid-packet reception
    
    // User code 
    ticktime = micros();
    // Serial.println("tick " + String(ticktime - tocktime));
   
    updatePhaseLock();
    OtaNonce++;
    
    // ELRS 自己注释的
    // if (!alreadyTLMresp && !alreadyFHSS && !LQCalc.currentIsSet()) // packet timeout AND didn't DIDN'T just hop or send TLM
    // {
    //     Radio.RXnb(); // put the radio cleanly back into RX in case of garbage data
    // }


    // if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    // {
    //     // Save the LQ value before the inc() reduces it by 1
    //     uplinkLQ = LQCalc.getLQ();
    // } else
    // if (!((OtaNonce - 1) % ExpressLRS_currAirRate_Modparams->numOfSends))
    // {
    //     uplinkLQ = LQCalcDVDA.getLQ();
    //     LQCalcDVDA.inc();
    // }

    // CRSF::LinkStatistics.uplink_Link_quality = uplinkLQ;
    // // Only advance the LQI period counter if we didn't send Telemetry this period
    // if (!alreadyTLMresp)
    //     LQCalc.inc();

    alreadyTLMresp = false;
    alreadyFHSS = false;
}

void ICACHE_RAM_ATTR HWtimerCallbackTock() // ELRS移植，注释源码另起修改
{
    // User code
    tocktime = micros();
    slack = tocktime - RxISRtime;
    // if(slack > 300)
        // Serial.println("SLACK " + String(slack));

    PFDloop.intEvent(micros()); // our internal osc just fired

    // if (ExpressLRS_currAirRate_Modparams->numOfSends > 1 && !(OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends))
    // {
    //     if (LQCalcDVDA.currentIsSet())
    //     {
    //         crsfRCFrameAvailable();
    //         if (teamraceHasModelMatch)
    //             servoNewChannelsAvailable();
    //     }
    //     else
    //     {
    //         crsfRCFrameMissed();
    //     }
    // }
    // else if (ExpressLRS_currAirRate_Modparams->numOfSends == 1)
    // {
    //     if (!LQCalc.currentIsSet())
    //     {
    //         crsfRCFrameMissed();
    //     }
    // }

    // // For any serial drivers that need to send on a regular cadence (i.e. CRSF to betaflight)
    // sendImmediateRC();

    // 如果因为在RXISR中因为CRC错误导致没有handleFHSS则在TOCK中处理
    if (!didFHSS)
    {
        HandleFHSS();
    }
    didFHSS = false;

    Radio.isFirstRxIrq = true;
    // updateDiversity();
    // tlmSent = HandleSendTelemetryResponse();

    #if defined(DEBUG_RX_SCOREBOARD)
    static bool lastPacketWasTelemetry = false;
    if (!LQCalc.currentIsSet() && !lastPacketWasTelemetry)
        DBGW(lastPacketCrcError ? '.' : '_');
    lastPacketCrcError = false;
    lastPacketWasTelemetry = tlmSent;
    #endif
}

// void LostConnection(bool resumeRx) // ELRS移植，注释源码另起修改
// {
//     Serial.println("lost conn fc= " + String(FreqCorrection) + " fo= " + String(hwTimer::getFreqOffset()));
//     // DBGLN("lost conn fc=%d fo=%d", FreqCorrection, hwTimer::getFreqOffset());
//
//     // // Use this rate as the initial rate next time if we connected on it
//     // if (connectionState == connected)
//     //     config.SetRateInitialIdx(ExpressLRS_nextAirRateIndex);
//
//     RFmodeCycleMultiplier = 1;
//     connectionState = disconnected; //set lost connection
//     RXtimerState = tim_disconnected;
//     hwTimer::resetFreqOffset();
//     PfdPrevRawOffset = 0;
//     GotConnectionMillis = 0;
//     uplinkLQ = 0;
//     LQCalc.reset();
//     LQCalcDVDA.reset();
//     LPF_Offset.init(0);
//     LPF_OffsetDx.init(0);
//     alreadyTLMresp = false;
//     alreadyFHSS = false;
//
//     if (!InBindingMode)
//     {
//         if (hwTimer::running)
//         {
//             while(micros() - PFDloop.getIntEventTime() > 250); // time it just after the tock()
//             hwTimer::stop();
//         }
//         SetRFLinkRate(ExpressLRS_nextAirRateIndex, false); // also sets to initialFreq
//         // If not resumRx, Radio will be left in SX127x_OPMODE_STANDBY / SX1280_MODE_STDBY_XOSC
//         if (resumeRx)
//         {
//             Radio.RXnb();
//         }
//     }
// }
//
// void ICACHE_RAM_ATTR TentativeConnection(unsigned long now) // ELRS移植，注释源码另起修改
// {
//     PFDloop.reset();
//     connectionState = tentative;
//     connectionHasModelMatch = false;
//     RXtimerState = tim_disconnected;
//     Serial.println("tentative conn");
//     // DBGLN("tentative conn");
//     PfdPrevRawOffset = 0;
//     LPF_Offset.init(0);
//     SnrMean.reset();
//     RFmodeLastCycled = now; // give another 3 sec for lock to occur
//
//     // The caller MUST call hwTimer::resume(). It is not done here because
//     // the timer ISR will fire immediately and preempt any other code
// }
//
// void GotConnection(unsigned long now) // ELRS移植，注释源码另起修改
// {
//     if (connectionState == connected)
//     {
//         return; // Already connected
//     }
//
//     // LockRFmode = firmwareOptions.lock_on_first_connection;
//
//     connectionState = connected; //we got a packet, therefore no lost connection
//     RXtimerState = tim_tentative;
//     GotConnectionMillis = now;
//     // #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
//     // webserverPreventAutoStart = true;
//     // #endif
//
//     // if (firmwareOptions.is_airport)
//     // {
//     //     apInputBuffer.flush();
//     //     apOutputBuffer.flush();
//     // }
//
//     Serial.println("got conn");
//     // DBGLN("got conn");
// }

void ICACHE_RAM_ATTR OnELRSBindMSP(uint8_t* newUid4) // ELRS移植，注释源码另起修改
{
    // Binding over MSP only contains 4 bytes due to packet size limitations, clear out any leading bytes
    UID[0] = 0;
    UID[1] = 0;
    for (unsigned i = 0; i < 4; i++)
    {
        UID[i + 2] = newUid4[i];
    }

    Serial.print("New UID ");
    for (int i = 0; i < sizeof(UID); i++)
        Serial.print(String(UID[i]) + " ");
    Serial.println();
    // DBGLN("New UID = %u, %u, %u, %u, %u, %u", UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

    // Set new UID in eeprom
    // EEPROM commit will happen on the main thread in ExitBindingMode()
    eeprom.Put(0, UID);
    // config.SetUID(UID);
    UIDIsModified = true; // 取代 config.IsModified()
}

static void ICACHE_RAM_ATTR ProcessRfPacket_MSP(OTA_Packet_s const * const otaPktPtr) // ELRS移植，注释源码另起修改
{
    uint8_t packageIndex;
    uint8_t const * payload;
    uint8_t dataLen;
    if (OtaIsFullRes)
    {
        packageIndex = otaPktPtr->full.msp_ul.packageIndex;
        payload = otaPktPtr->full.msp_ul.payload;
        dataLen = sizeof(otaPktPtr->full.msp_ul.payload);
        // if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        // {
        //     TelemetrySender.ConfirmCurrentPayload(otaPktPtr->full.msp_ul.tlmFlag);
        // }
        // else
        {
            packageIndex &= ELRS8_TELEMETRY_MAX_PACKAGES;
        }
    }
    else
    {
        packageIndex = otaPktPtr->std.msp_ul.packageIndex;
        payload = otaPktPtr->std.msp_ul.payload;
        dataLen = sizeof(otaPktPtr->std.msp_ul.payload);
        // // User code--show ota msp data
        // Serial.print("recieved data: ");
        // for(int i = 0; i < dataLen; i++)
        // {
        //     Serial.print((char)*(payload + i));
        // }
        // Serial.println("");

        // if (config.GetSerialProtocol() == PROTOCOL_MAVLINK)
        // {
        //     TelemetrySender.ConfirmCurrentPayload(otaPktPtr->std.msp_ul.tlmFlag);
        // }
        // else
        {
            packageIndex &= ELRS4_TELEMETRY_MAX_PACKAGES;
        }
    }

    // Always examine MSP packets for bind information if in bind mode
    // [1] is the package index, first packet of the MSP
    if (InBindingMode && packageIndex == 1 && payload[0] == MSP_ELRS_BIND)
    {
        OnELRSBindMSP((uint8_t *)&payload[1]);
        return;
    }

    // // Must be fully connected to process MSP, prevents processing MSP
    // // during sync, where packets can be received before connection
    // if (connectionState != connected)
    //     return;

    // bool currentMspConfirmValue = MspReceiver.GetCurrentConfirm();
    // MspReceiver.ReceiveData(packageIndex, payload, dataLen);
    // if (currentMspConfirmValue != MspReceiver.GetCurrentConfirm())
    // {
    //     NextTelemetryType = ELRS_TELEMETRY_TYPE_LINK;
    // }
}

static bool ICACHE_RAM_ATTR ProcessRfPacket_SYNC(uint32_t const now, OTA_Sync_s const * const otaSync)
{
    // Verify the first two of three bytes of the binding ID, which should always match
    if (otaSync->UID3 != UID[3] || otaSync->UID4 != UID[4])
        return false;

    // // The third byte will be XORed with inverse of the ModelId if ModelMatch is on
    // // Only require the first 18 bits of the UID to match to establish a connection
    // // but the last 6 bits must modelmatch before sending any data to the FC
    // if ((otaSync->UID5 & ~MODELMATCH_MASK) != (UID[5] & ~MODELMATCH_MASK))
    //     return false;

    LastSyncPacket = now;
#if defined(DEBUG_RX_SCOREBOARD)
    DBGW('s');
#endif

    // // Will change the packet air rate in loop() if this changes
    // ExpressLRS_nextAirRateIndex = otaSync->rateIndex;
    // updateSwitchModePendingFromOta(otaSync->switchEncMode);

    // Update TLM ratio, should never be TLM_RATIO_STD/DISARMED, the TX calculates the correct value for the RX
    // expresslrs_tlm_ratio_e TLMrateIn = (expresslrs_tlm_ratio_e)(otaSync->newTlmRatio + (uint8_t)TLM_RATIO_NO_TLM);
    // uint8_t TlmDenom = TLMratioEnumToValue(TLMrateIn);
    // if (ExpressLRS_currTlmDenom != TlmDenom)
    // {
    //     DBGLN("New TLMrate 1:%u", TlmDenom);
    //     ExpressLRS_currTlmDenom = TlmDenom;
    //     telemBurstValid = false;
    // }

    // // modelId = 0xff indicates modelMatch is disabled, the XOR does nothing in that case
    // uint8_t modelXor = (~config.GetModelId()) & MODELMATCH_MASK;
    // bool modelMatched = otaSync->UID5 == (UID[5] ^ modelXor);
    // DBGVLN("MM %u=%u %d", otaSync->UID5, UID[5], modelMatched);

    if (connectionState == disconnected
        || OtaNonce != otaSync->nonce
        || FHSSgetCurrIndex() != otaSync->fhssIndex
        // || connectionHasModelMatch != modelMatched
        )
    {
        //DBGLN("\r\n%ux%ux%u", OtaNonce, otaPktPtr->sync.nonce, otaPktPtr->sync.fhssIndex);
        FHSSsetCurrIndex(otaSync->fhssIndex);
        OtaNonce = otaSync->nonce;
        // TentativeConnection(now);
        connectionState = connected;
        // // connectionHasModelMatch must come after TentativeConnection, which resets it
        // connectionHasModelMatch = modelMatched;
        return true;
    }

    return false;
}

bool ICACHE_RAM_ATTR ProcessRFPacket(SX12xxDriverCommon::rx_status const status) // ELRS移植，注释源码另起修改
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK)
    {
        Serial.println("HW CRC error");
        // DBGVLN("HW CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }
    uint32_t const beginProcessing = micros();

    OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;
    CRCvalue = otaPktPtr->std.crcLow | otaPktPtr->std.crcHigh;
    if (!OtaValidatePacketCrc(otaPktPtr))
    {
        Serial.println("CRC error");
        // DBGVLN("CRC error");
        #if defined(DEBUG_RX_SCOREBOARD)
            lastPacketCrcError = true;
        #endif
        return false;
    }
    receivecount++;
    PFDloop.extEvent(beginProcessing + PACKET_TO_TOCK_SLACK);

    doStartTimer = false;
    unsigned long now = millis();

    LastValidPacket = now;

    switch (otaPktPtr->std.type)
    {
    case PACKET_TYPE_RCDATA: //Standard RC Data Packet
        // ProcessRfPacket_RC(otaPktPtr);
        break;
    case PACKET_TYPE_MSPDATA:
        ProcessRfPacket_MSP(otaPktPtr);
        break;
    case PACKET_TYPE_SYNC: //sync packet from master
        doStartTimer = ProcessRfPacket_SYNC(now,
            OtaIsFullRes ? &otaPktPtr->full.sync.sync : &otaPktPtr->std.sync)
            && !InBindingMode;
        break;
    // case PACKET_TYPE_TLM:
    //     if (firmwareOptions.is_airport)
    //     {
    //         OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
    //     }
    //     break;
    default:
        break;
    }

    // // Store the LQ/RSSI/Antenna
    // Radio.GetLastPacketStats();
    // getRFlinkInfo();

    if (Radio.FrequencyErrorAvailable())
    {
    #if defined(RADIO_SX127X)
        // Adjusts FreqCorrection for RX freq offset
        int32_t tempFreqCorrection = HandleFreqCorr(Radio.GetFrequencyErrorbool());
        // Teamp900 also needs to adjust its demood PPM
        Radio.SetPPMoffsetReg(tempFreqCorrection);
    #else /* !RADIO_SX127X */
        // Adjusts FreqCorrection for RX freq offset
        HandleFreqCorr(Radio.GetFrequencyErrorbool());
    #endif /* RADIO_SX127X */
    }

    // // Received a packet, that's the definition of LQ
    // LQCalc.add();
    // // Extend sync duration since we've received a packet at this rate
    // // but do not extend it indefinitely
    // RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow;

#if defined(DEBUG_RX_SCOREBOARD)
    if (otaPktPtr->std.type != PACKET_TYPE_SYNC) DBGW(connectionHasModelMatch ? 'R' : 'r');
#endif

    return true;
}

bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status) // ELRS移植，注释源码另起修改
{
    // User code 
    RxISRtime = micros();
    // Serial.println("RXdoneISR " + String(RxISRtime));

    // if (LQCalc.currentIsSet() && connectionState == connected)
    // {
    //     return false; // Already received a packet, do not run ProcessRFPacket() again.
    // }

    if (ProcessRFPacket(status))
    {
        didFHSS = HandleFHSS();

        // if (doStartTimer)
        // {
        //     doStartTimer = false;
        //     hwTimer::resume(); // will throw an interrupt immediately
        // }

        return true;
    }
    return false;
}

static void setupBindingFromConfig() // ELRS移植，注释源码另起修改
{
    // // VolatileBind's only function is to prevent loading the stored UID into RAM
    // // which makes the RX boot into bind mode every time
    // if (config.GetIsBound())
    // {
    //     memcpy(UID, config.GetUID(), UID_LEN);
    // }
    eeprom.Begin();
    eeprom.Get(0, UID);

    Serial.print("UID ");
    for (int i = 0; i < sizeof(UID); i++)
        Serial.print(String(UID[i]) + " ");
    Serial.println();
    // DBGLN("UID=(%d, %d, %d, %d, %d, %d) ModelId=%u",
    //     UID[0], UID[1], UID[2], UID[3], UID[4], UID[5], config.GetModelId());

    OtaUpdateCrcInitFromUid();
}

static void setupRadio() // ELRS移植，注释源码另起修改
{
    Radio.currFreq = FHSSgetInitialFreq();
#if defined(RADIO_SX127X)
    //Radio.currSyncWord = UID[3];
#endif
    bool init_success = Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());
    // POWERMGNT::init();
    if (!init_success)
    {
        Serial.println("Failed to detect RF chipset!!!");
        connectionState = radioFailed;
        return;
    }

    // DynamicPower_UpdateRx(true);

#if defined(Regulatory_Domain_EU_CE_2400)
    LBTEnabled = (config.GetPower() > PWR_10mW);
#endif

    Radio.RXdoneCallback = &RXdoneISR;
    // Radio.TXdoneCallback = &TXdoneISR;

    SetRFLinkRate(enumRatetoIndex(airRate), false);
    // scanIndex = config.GetRateInitialIdx();
    // SetRFLinkRate(scanIndex, false);
    // // Start slow on the selected rate to give it the best chance
    // // to connect before beginning rate cycling
    // RFmodeCycleMultiplier = RFmodeCycleMultiplierSlow / 2;
}

/* If not connected will rotate through the RF modes looking for sync
 * and blink LED
 */
static void cycleRfMode(unsigned long now)
{
    if (connectionState == connected || connectionState == wifiUpdate || InBindingMode)
        return;

    // Actually cycle the RF mode if not LOCK_ON_FIRST_CONNECTION
    if (LockRFmode == false && (now - RFmodeLastCycled) > (cycleInterval * RFmodeCycleMultiplier))
    {
        RFmodeLastCycled = now;
        LastSyncPacket = now;           // reset this variable
        // SendLinkStatstoFCForcedSends = 2;
        SetRFLinkRate(scanIndex % RATE_MAX, false); // switch between rates
        LQCalc.reset100();
        LQCalcDVDA.reset100();
        // Display the current air rate to the user as an indicator something is happening
        scanIndex++;
        Radio.RXnb();

        Serial.println(String(ExpressLRS_currAirRate_Modparams->interval));
        // DBGLN("%u", ExpressLRS_currAirRate_Modparams->interval);

        // Switch to FAST_SYNC if not already in it (won't be if was just connected)
        RFmodeCycleMultiplier = 1;
    } // if time to switch RF mode
}

static void EnterBindingMode() // ELRS移植，注释源码另起修改
{
    if (InBindingMode)
    {
        // DBGLN("Already in binding mode");
        Serial.println("Already in binding mode");
        return;
    }
    // 点亮LED
    digitalWrite(PC13, 0);
    // Binding uses a CRCInit=0, 50Hz, and InvertIQ
    OtaCrcInitializer = 0;
    InBindingMode = true;
    // // Any method of entering bind resets a loan
    // // Model can be reloaned immediately by binding now
    // config.ReturnLoan();
    // config.Commit();

    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(enumRatetoIndex(RATE_BINDING), true);

    // If the Radio Params (including InvertIQ) parameter changed, need to restart RX to take effect
    Radio.RXnb(SX1280_MODE_RX_CONT);
    // Radio.RXnb();

    Serial.println("Entered binding mode at freq = " + String(Radio.currFreq));
    // DBGLN("Entered binding mode at freq = %d", Radio.currFreq);
    // devicesTriggerEvent();
}

static void ExitBindingMode() // ELRS移植，注释源码另起修改
{
    if (!InBindingMode)
    {
        // DBGLN("Not in binding mode");
        Serial.println("Not in binding mode");
        return;
    }
    // 熄灭LED
    digitalWrite(PC13, 1);
    // MspReceiver.ResetState();

    // Prevent any new packets from coming in
    Radio.SetTxIdleMode();
    // Write the values to eeprom
    eeprom.Commit();
    // config.Commit();

    OtaUpdateCrcInitFromUid();
    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    webserverPreventAutoStart = true;
    #endif

    SetRFLinkRate(enumRatetoIndex(airRate), false);
    Radio.RXnb();
    // // Force RF cycling to start at the beginning immediately
    // scanIndex = RATE_MAX;
    // RFmodeLastCycled = 0;
    // LockRFmode = false;
    // LostConnection(false);

    // Do this last as LostConnection() will wait for a tock that never comes
    // if we're in binding mode
    InBindingMode = false;
    Serial.println("Exiting binding mode");
    // DBGLN("Exiting binding mode");
    // devicesTriggerEvent();
}

static void updateBindingMode(unsigned long now) // ELRS移植，注释源码另起修改
{
    // Exit binding mode if the config has been modified, indicating UID has been set
    if (InBindingMode && /*config.IsModified()*/ UIDIsModified/*自定义变量*/)
    {
        Serial.println("UID modified only once");
        ExitBindingMode();
    }

#if defined(RADIO_LR1121)
    // Change frequency domains every 500ms.  This will allow single LR1121 receivers to receive bind packets from SX12XX Tx modules.
    else if (InBindingMode && (now - BindingRateChangeTime) > BindingRateChangeCyclePeriod)
    {
        BindingRateChangeTime = now;
        if (ExpressLRS_currAirRate_Modparams->index == RATE_DUALBAND_BINDING)
        {
            SetRFLinkRate(enumRatetoIndex(RATE_BINDING), true);
        }
        else
        {
            SetRFLinkRate(RATE_DUALBAND_BINDING, true);
        }

        Radio.RXnb();
    }
#endif

//     // If the power on counter is >=3, enter binding, the counter will be reset after 2s
//     else if (!InBindingMode && config.GetPowerOnCounter() >= 3)
//     {
// #if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
//         // Never enter wifi if forced to binding mode
//         webserverPreventAutoStart = true;
// #endif
//         DBGLN("Power on counter >=3, enter binding mode");
//         EnterBindingMode();
//     }

    // If the eeprom is indicating that we're not bound, enter binding
    else if (!UID_IS_BOUND(UID) && !InBindingMode)
    {
        // DBGLN("RX has not been bound, enter binding mode");
        Serial.println("RX has not been bound, enter binding mode");
        EnterBindingMode();
    }

    else if (BindingModeRequest)
    {
        // DBGLN("Connected request to enter binding mode");
        Serial.println("Connected request to enter binding mode");
        BindingModeRequest = false;
        // if (connectionState == connected)
        // {
        //     LostConnection(false);
        //     // Skip entering bind mode if on loan. This comes from an OTA request
        //     // and the model is assumed to be inaccessible, do not want the receiver
        //     // sitting in a field ready to be bound to anyone within 10km
        //     if (config.IsOnLoan())
        //     {
        //         DBGLN("Model was on loan, becoming inert");
        //         config.ReturnLoan();
        //         config.Commit(); // prevents CheckConfigChangePending() re-enabling radio
        //         Radio.End();
        //         // Enter a completely invalid state for a receiver, to prevent wifi or radio enabling
        //         connectionState = noCrossfire;
        //         return;
        //     }
        //     // if the InitRate config item was changed by LostConnection
        //     // save the config before entering bind, as the modified config
        //     // will immediately boot it out of bind mode
        //     config.Commit();
        // }
        EnterBindingMode();
    }
}

void EnterBindingModeSafely() // ELRS移植，注释源码另起修改
{
    // Will not enter Binding mode if in the process of a passthrough update
    // or currently binding
    if (connectionState == serialUpdate || InBindingMode)
        return;

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)
    // Never enter wifi mode after requesting to enter binding mode
    webserverPreventAutoStart = true;

    // If the radio and everything is shut down, better to reboot and boot to binding mode
    if (connectionState == wifiUpdate || connectionState == bleJoystick)
    {
        // Force 3-plug binding mode
        config.SetPowerOnCounter(3);
        config.Commit();
        ESP.restart();
        // Unreachable
    }
#endif

    // // If connected, handle that in updateBindingMode()
    // if (connectionState == connected)
    // {
    //     BindingModeRequest = true;
    //     return;
    // }

    EnterBindingMode();
}

/* User Function*/

void displayDebugInfo()
{
    display.clearDisplay();  
    if(InBindingMode)
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
        // receive rate
        display.setCursor(0, 24);   
        display.println("Rate");     
        display.setCursor(30, 24);         
        display.println(receivefreq);  
    }
    // CRC
    display.setCursor(0, 16);
    display.println("CRC");
    display.setCursor(24, 16);
    display.println(CRCvalue);
    // Freq
    display.setCursor(0, 8);           
    display.println("FQ");    
    display.setCursor(18, 8);           
    display.println(Radio.currFreq);  
    // Channel
    display.setCursor(76, 8);           
    display.println("CH");  
    display.setCursor(94, 8);           
    display.println(FHSSgetCurrIndex());  
    // RSSI
    display.setCursor(54, 24);     
    display.println("RSSI");     
    display.setCursor(84, 24);           
    display.println(Radio.GetRssiInst(SX12XX_Radio_1)); 
    display.display();
}

void handleButtonPress() 
{
    delay(25);
    if (digitalRead(PB1) == LOW)
    {
        while (digitalRead(PB1) == LOW)
            ;
        delay(25);
        EnterBindingModeSafely();
    }
}

void TimerHandler() 
{  
    receivefreq = receivecount;
    receivecount = 0;
}

void setupBasicHardWare()
{
    // UART
    Serial.begin(420000);
    // LED
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH);
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
    // sx1280 GPIO
    pinMode(GPIO_PIN_TX_EN, OUTPUT);
    pinMode(GPIO_PIN_RX_EN, OUTPUT);
    // TIM2
    ITimer.attachInterruptInterval(TIMER_INTERVAL_MS, TimerHandler);
}

static void HandleUARTin()
{
    if(Serial.available())
    {
        auto size = std::min(apInputBuffer.free(), (uint16_t)Serial.available());
        if (size > 0)
        {
            uint8_t buf[size];
            Serial.readBytes(buf, size);
            apInputBuffer.lock();
            apInputBuffer.pushBytes(buf, size);
            apInputBuffer.unlock();
        }
    }
}

static void HandleUARTout()
{
    if(Serial.availableForWrite())
    {
        auto size = apOutputBuffer.size();
        if (size)
        {
            uint8_t buf[size];
            apOutputBuffer.lock();
            apOutputBuffer.popBytes(buf, size);
            apOutputBuffer.unlock();
            Serial.write(buf, size);
        }
    }
}

/* setup and loop */

void setup() // 与ELRS初始化逻辑一致
{
    // 删除ELRS硬件初始化代码，适配BluePill硬件平台
    setupBasicHardWare();

    setupBindingFromConfig();

    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    setupRadio();

    if (connectionState != radioFailed)
    {
        // MspReceiver.SetDataToReceive(MspData, ELRS_MSP_BUFFER);
        Radio.RXnb(SX1280_MODE_RX_CONT);
        hwTimer::init(HWtimerCallbackTick, HWtimerCallbackTock);
    }
    hwTimer::resume();

    // setup() eats up some of this time, which can cause the first mode connection to fail.
    // Resetting the time here give the first mode a better chance of connection.
    RFmodeLastCycled = millis();
}

void loop() // ELRS移植，注释源码另起修改
{
    unsigned long now = millis();
    // disconnect
    if(slack / ExpressLRS_currAirRate_Modparams->interval > 5)
    {
        // Serial.println("lost connection");
        connectionState = disconnected;
        Radio.SetFrequencyReg(FHSSgetInitialFreq());
    }
    // if (MspReceiver.HasFinishedData())
    // {
    //     MspReceiveComplete();
    // }

    // devicesUpdate(now);

    // read and process any data from serial ports, send any queued non-RC data
    // handleSerialIO();
    HandleUARTin();
    HandleUARTout();

    // CheckConfigChangePending();
    // executeDeferredFunction(micros());

    // // Clear the power-on-count
    // if ((connectionState == connected || connectionState == tentative) && config.GetPowerOnCounter() != 0)
    // {
    //     config.SetPowerOnCounter(0);
    // }

    // if (connectionState > MODE_STATES)
    // {
    //     return;
    // }

    // if ((connectionState != disconnected) && (ExpressLRS_currAirRate_Modparams->index != ExpressLRS_nextAirRateIndex)){ // forced change
    //     Serial.println("Req air rate change " + String(ExpressLRS_currAirRate_Modparams->index) + " -> " + String(ExpressLRS_nextAirRateIndex));
    //     // DBGLN("Req air rate change %u->%u", ExpressLRS_currAirRate_Modparams->index, ExpressLRS_nextAirRateIndex);
    //     LostConnection(true);
    //     LastSyncPacket = now;           // reset this variable to stop rf mode switching and add extra time
    //     RFmodeLastCycled = now;         // reset this variable to stop rf mode switching and add extra time
    //     // SendLinkStatstoFCintervalLastSent = 0;
    //     // SendLinkStatstoFCForcedSends = 2;
    // }

    // if (connectionState == tentative && (now - LastSyncPacket > ExpressLRS_currAirRate_RFperfParams->RxLockTimeoutMs))
    // {
    //     Serial.println("Bad sync, aborting");
    //     // DBGLN("Bad sync, aborting");
    //     LostConnection(true);
    //     RFmodeLastCycled = now;
    //     LastSyncPacket = now;
    // }

    // // cycleRfMode(now);

    // uint32_t localLastValidPacket = LastValidPacket; // Required to prevent race condition due to LastValidPacket getting updated from ISR
    // if ((connectionState == connected) && ((int32_t)ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs < (int32_t)(now - localLastValidPacket))) // check if we lost conn.
    // {
    //     LostConnection(true);
    // }

    // if ((connectionState == tentative) && (abs(LPF_OffsetDx.value()) <= 10) && (LPF_Offset.value() < 100) && (LQCalc.getLQRaw() > minLqForChaos())) //detects when we are connected
    // {
    //     GotConnection(now);
    // }

    // // checkSendLinkStatsToFc(now);

    // if ((RXtimerState == tim_tentative) && ((now - GotConnectionMillis) > ConsiderConnGoodMillis) && (abs(LPF_OffsetDx.value()) <= 5))
    // {
    //     RXtimerState = tim_locked;
    //     // DBGLN("Timer locked");
    //     Serial.println("Timer locked");
    // }

    // uint8_t *nextPayload = 0;
    // uint8_t nextPlayloadSize = 0;
    // if (!TelemetrySender.IsActive() && telemetry.GetNextPayload(&nextPlayloadSize, &nextPayload))
    // {
    //     TelemetrySender.SetDataToTransmit(nextPayload, nextPlayloadSize);
    // }

    // uint16_t count = mavlinkInputBuffer.size();
    // if (count > 0 && !TelemetrySender.IsActive())
    // {
    //     count = std::min(count, (uint16_t)CRSF_PAYLOAD_SIZE_MAX); // Constrain to CRSF max payload size to match SS
    //     // First 2 bytes conform to crsf_header_s format
    //     mavlinkSSBuffer[0] = CRSF_ADDRESS_USB; // device_addr - used on TX to differentiate between std tlm and mavlink
    //     mavlinkSSBuffer[1] = count;
    //     // Following n bytes are just raw mavlink
    //     mavlinkInputBuffer.popBytes(mavlinkSSBuffer + CRSF_FRAME_NOT_COUNTED_BYTES, count);
    //     nextPayload = mavlinkSSBuffer;
    //     nextPlayloadSize = count + CRSF_FRAME_NOT_COUNTED_BYTES;
    //     TelemetrySender.SetDataToTransmit(nextPayload, nextPlayloadSize);
    // }

    // updateTelemetryBurst();
    updateBindingMode(now);
    // updateSwitchMode();
    // checkGeminiMode();
    // DynamicPower_UpdateRx(false);
    // debugRcvrLinkstats();
    // debugRcvrSignalStats(now);
    
    displayDebugInfo();
}
