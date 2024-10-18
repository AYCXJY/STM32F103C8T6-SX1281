// 只使用OTA包发送UID，

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

// User //

// OLED设置
#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(OLED_RESET);
// 空中速率
#define airRate RATE_LORA_500HZ
// 显示实际发包数
uint16_t sendcount;
uint16_t sendfreq;

// ELRS //

// 绑定包发送次数和计数变量
#define BindingSpamAmount 25
static uint8_t BindingSendCount;
// 链路质量
// LQCALC<25> LQCalc;
// 收发模式切换状态机
// static TxTlmRcvPhase_e TelemetryRcvPhase = ttrpTransmitting;
// TelemetryReceiver 将接收到的数据放入 CRSFinBuffer
// StubbornReceiver TelemetryReceiver;
// uint8_t CRSFinBuffer[CRSF_MAX_PACKET_LEN+1];
// 将发送的数据赋予它
// StubbornSender MspSender;

// #define syncSpamAmount 3
// #define syncSpamAmountAfterRateChange 10
// volatile uint8_t syncSpamCounter = 0;
// volatile uint8_t syncSpamCounterAfterRateChange = 0;
// uint32_t rfModeLastChangedMS = 0;
// uint32_t SyncPacketLastSent = 0;

void SetRFLinkRate(uint8_t index) // 无关删除 功能保留
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
    // Binding always uses invertIQ
    bool invertIQ = InBindingMode || (UID[5] & 0x01);
    // OtaSwitchMode_e newSwitchMode = (OtaSwitchMode_e)config.GetSwitchMode();

    if ((ModParams == ExpressLRS_currAirRate_Modparams) && (RFperf == ExpressLRS_currAirRate_RFperfParams) && (invertIQ == Radio.IQinverted)
        // && (OtaSwitchModeCurrent == newSwitchMode)
    )
    {
        Serial.println("same rate no process");
        return;
    }

    Serial.println("set rate " + String(index));
    uint32_t interval = ModParams->interval;

    hwTimer::updateInterval(interval);

    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

    Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
                 ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, ModParams->interval, uidMacSeedGet(), OtaCrcInitializer, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC));

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);

    // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
    FHSSsetCurrIndex(0);
    OtaNonce = 0;

    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
    // OtaUpdateSerializers(newSwitchMode, ModParams->PayloadLength);

    // MspSender.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
    // TelemetryReceiver.setMaxPackageIndex(OtaIsFullRes ? ELRS8_TELEMETRY_MAX_PACKAGES : ELRS4_TELEMETRY_MAX_PACKAGES);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    // CRSF::LinkStatistics.rf_Mode = ModParams->enum_rate;

    // handset->setPacketInterval(interval * ExpressLRS_currAirRate_Modparams->numOfSends);
    // connectionState = disconnected;
    // rfModeLastChangedMS = millis();
}

static void setupBindingFromConfig() // 无关删除 功能保留
{
    // if (firmwareOptions.hasUID)
    // {
    //     memcpy(UID, firmwareOptions.uid, UID_LEN);
    // }
    // else
    // {
    UID[0] = (uint8_t)HAL_GetUIDw0();
    UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
    UID[2] = (uint8_t)HAL_GetUIDw1();
    UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
    UID[4] = (uint8_t)HAL_GetUIDw2();
    UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);
    // }
    Serial.print("UID ");
    for (int i = 0; i < sizeof(UID); i++)
        Serial.print(String(UID[i]) + " ");
    Serial.println();
    // UID 变化的重新计算校验种子
    OtaUpdateCrcInitFromUid();
}

static void ExitBindingMode() // 无关删除 功能保留
{
    if (!InBindingMode)
        return;

    // MspSender.ResetState();

    // Reset CRCInit to UID-defined value
    OtaUpdateCrcInitFromUid();
    // Clear binding mode before SetRFLinkRate() for correct IQ
    InBindingMode = false;
    // return to original rate
    SetRFLinkRate(enumRatetoIndex(airRate));
    // SetRFLinkRate(config.GetRate());

    Serial.println("Exiting binding mode");
}

void SendUIDOverMSP() // 无变动
{
    uint8_t MSPDataPackage[5];
    MSPDataPackage[0] = MSP_ELRS_BIND;
    memcpy(&MSPDataPackage[1], &UID[2], 4);
    BindingSendCount = 0;
    // MspSender.ResetState();
    // MspSender.SetDataToTransmit(MSPDataPackage, 5);
}

static void EnterBindingMode() // 已同步
{
    if (InBindingMode)
        return;
    // Disable the TX timer and wait for any TX to complete
    hwTimer::stop();
    // Queue up sending the Master UID as MSP packets
    SendUIDOverMSP();
    // Binding uses a CRCInit=0, 50Hz, and InvertIQ
    OtaCrcInitializer = 0;

    // Lock the OtaNonce to prevent syncspam packets
    OtaNonce = 0;
    // Set binding mode before SetRFLinkRate() for correct IQ
    InBindingMode = true;
    // Start attempting to bind
    // Lock the RF rate and freq while binding
    SetRFLinkRate(enumRatetoIndex(RATE_BINDING));
    Serial.println("Entered binding mode at freq = " + String(Radio.currFreq));
    // Start transmitting again
    hwTimer::resume();
}

void ICACHE_RAM_ATTR HandleFHSS() // 已同步
{
    uint8_t modresult = (OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
    // If the next packet should be on the next FHSS frequency, do the hop
    if (!InBindingMode && modresult == 0)
    {
        Radio.SetFrequencyReg(FHSSgetNextFreq());
    }
}

void ICACHE_RAM_ATTR TXdoneISR() // 已同步
{
    HandleFHSS();

    // HandlePrepareForTLM();
    sendcount++;
}

void ICACHE_RAM_ATTR SendRCdataToRF() // 无关删除 功能保留
{
    //   // Do not send a stale channels packet to the RX if one has not been received from the handset
    //   // *Do* send data if a packet has never been received from handset and the timer is running
    //   // this is the case when bench testing and TXing without a handset
    //   bool dontSendChannelData = false;
    //   uint32_t lastRcData = handset->GetRCdataLastRecv();
    //   if (lastRcData && (micros() - lastRcData > 1000000))
    //   {
    //     The tx is in Mavlink mode and without a valid crsf or RC input.  Do not send stale or fake zero packet RC!
    //     Only send sync and MSP packets.
    //     if (config.GetLinkMode() == TX_MAVLINK_MODE)
    //     {
    //       dontSendChannelData = true;
    //     }
    //     else
    //     {
    //       return;
    //     }
    //   }

    // uint32_t const now = millis();
    // ESP requires word aligned buffer
    WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
    //   static uint8_t syncSlot;

    //   const bool isTlmDisarmed = config.GetTlm() == TLM_RATIO_DISARMED;
    //   uint32_t SyncInterval = (connectionState == connected && !isTlmDisarmed) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
    //   bool skipSync = InBindingMode ||
    //     // TLM_RATIO_DISARMED keeps sending sync packets even when armed until the RX stops sending telemetry and the TLM=Off has taken effect
    //     (isTlmDisarmed && handset->IsArmed() && (ExpressLRS_currTlmDenom == 1));

    // uint8_t NonceFHSSresult = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

    //   // Sync spam only happens on slot 1 and 2 and can't be disabled
    //   if ((syncSpamCounter || (syncSpamCounterAfterRateChange && FHSSonSyncChannel())) && (NonceFHSSresult == 1 || NonceFHSSresult == 2))
    //   {
    //     otaPkt.std.type = PACKET_TYPE_SYNC;
    //     GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
    //     syncSlot = 0; // reset the sync slot in case the new rate (after the syncspam) has a lower FHSShopInterval
    //   }
    //   // Regular sync rotates through 4x slots, twice on each slot, and telemetry pushes it to the next slot up
    //   // But only on the sync FHSS channel and with a timed delay between them
    //   else if ((!skipSync) && ((syncSlot / 2) <= NonceFHSSresult) && (now - SyncPacketLastSent > SyncInterval) && FHSSonSyncChannel())
    //   {
    //     otaPkt.std.type = PACKET_TYPE_SYNC;
    //     GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
    //     syncSlot = (syncSlot + 1) % (ExpressLRS_currAirRate_Modparams->FHSShopInterval * 2);
    //   }
    //   else
    //   {
    // if ((/*NextPacketIsMspData && */MspSender.IsActive())/* || dontSendChannelData*/)
    {
        otaPkt.std.type = PACKET_TYPE_MSPDATA;
        //   if (OtaIsFullRes)
        //   {
        //     otaPkt.full.msp_ul.packageIndex = MspSender.GetCurrentPayload(
        //       otaPkt.full.msp_ul.payload,
        //       sizeof(otaPkt.full.msp_ul.payload));
        //     // if (config.GetLinkMode() == TX_MAVLINK_MODE)
        //     //   otaPkt.full.msp_ul.tlmFlag = TelemetryReceiver.GetCurrentConfirm();
        //   }
        //   else
        //   {
        otaPkt.std.msp_ul.packageIndex = 1;
        if (InBindingMode)
        {
            memcpy(&otaPkt.std.msp_ul.payload[0], &UID[2], 4);
        }
        else
        {
            otaPkt.std.msp_ul.payload[0] = 'h';
        }

        // otaPkt.std.msp_ul.packageIndex = MspSender.GetCurrentPayload(
        //   otaPkt.std.msp_ul.payload,
        //   sizeof(otaPkt.std.msp_ul.payload));
        // if (config.GetLinkMode() == TX_MAVLINK_MODE)
        //   otaPkt.std.msp_ul.tlmFlag = TelemetryReceiver.GetCurrentConfirm();
        //   }

        // send channel data next so the channel messages also get sent during msp transmissions
        //   NextPacketIsMspData = false;
        // counter can be increased even for normal msp messages since it's reset if a real bind message should be sent
        BindingSendCount++;
        // If the telemetry ratio isn't already 1:2, send a sync packet to boost it
        // to add bandwidth for the reply
        //   if (ExpressLRS_currTlmDenom != 2)
        //     syncSpamCounter = 1;
    }
    // else
    // {
    //   // always enable msp after a channel package since the slot is only used if MspSender has data to send
    //   NextPacketIsMspData = true;

    //   injectBackpackPanTiltRollData(now);
    //   OtaPackChannelData(&otaPkt, ChannelData, TelemetryReceiver.GetCurrentConfirm(), ExpressLRS_currTlmDenom);
    // }
    //   }
    OtaGeneratePacketCrc(&otaPkt);

    SX12XX_Radio_Number_t transmittingRadio = Radio.GetLastSuccessfulPacketRadio();

    Radio.TXnb((uint8_t *)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength, transmittingRadio);
}

void ICACHE_RAM_ATTR timerCallback() // 无关删除 功能保留
{
    // Nonce advances on every timer tick
    if (!InBindingMode)
        OtaNonce++;

    // // If HandleTLM has started Receive mode, TLM packet reception should begin shortly
    // // Skip transmitting on this slot
    // if (TelemetryRcvPhase == ttrpPreReceiveGap)
    // {
    //     TelemetryRcvPhase = ttrpExpectingTelem;
    //     CRSF::LinkStatistics.downlink_Link_quality = LQCalc.getLQ();
    //     LQCalc.inc();
    //     return;
    // }
    // TelemetryRcvPhase = ttrpTransmitting;

    static uint16_t tockcount = 0;
    tockcount++;
    if (tockcount >= (1000000 / ExpressLRS_currAirRate_Modparams->interval))
    {
        tockcount = 0;
        sendfreq = sendcount;
        sendcount = 0;
    }

    SendRCdataToRF();
}

void displayDebugInfo()
{
    display.clearDisplay();
    if (InBindingMode)
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
    display.println(Radio.currFreq);
    // Channel
    display.setCursor(76, 8);
    display.println("CH");
    display.setCursor(94, 8);
    display.println(FHSSgetCurrIndex());
    display.display();
}

void handleButtonPress(void)
{
    delay(25);
    if (digitalRead(PB1) == 0)
    {
        while (digitalRead(PB1) == 0)
            ;
        delay(25);
        digitalToggle(PC13);
        EnterBindingMode();
    }
}

void setupBasicHardWare(void)
{
    // 调试串口
    Serial.begin(420000);
    // 板载LED灯
    pinMode(PC13, OUTPUT);
    // 设置绑定按键
    pinMode(PB1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PB1), handleButtonPress, FALLING);
    // 初始化OLED调试屏
    Wire.setSCL(PB8);
    Wire.setSDA(PB9);
    Wire.begin();
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.display();
    // Radio 初始化使能发送接收位
    pinMode(GPIO_PIN_TX_EN, OUTPUT);
    pinMode(GPIO_PIN_RX_EN, OUTPUT);
}

void setup()
{
    setupBasicHardWare();

    setupBindingFromConfig();

    FHSSrandomiseFHSSsequence(uidMacSeedGet());

    // Radio.RXdoneCallback = &RXdoneISR;
    Radio.TXdoneCallback = &TXdoneISR;

    Radio.currFreq = FHSSgetInitialFreq();

    Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

    // TelemetryReceiver.SetDataToReceive(CRSFinBuffer, sizeof(CRSFinBuffer));

    SetRFLinkRate(enumRatetoIndex(airRate));

    hwTimer::init(nullptr, timerCallback);
    hwTimer::resume();
}

void loop()
{
    displayDebugInfo();
    // uint32_t now = millis();
    // only send msp data when binding is not active
    // static bool mspTransferActive = false;
    if (InBindingMode)
    {
        // exit bind mode if package after some repeats
        if (BindingSendCount > BindingSpamAmount)
        {
            ExitBindingMode();
        }
    }
    //     else if (!MspSender.IsActive())
    //     {
    //         // sending is done and we need to update our flag
    //         if (mspTransferActive)
    //         {
    //             // unlock buffer for msp messages
    //             CRSF::UnlockMspMessage();
    //             mspTransferActive = false;
    //         }
    //         // we are not sending so look for next msp package
    //         else
    //         {
    //             uint8_t* mspData;
    //             uint8_t mspLen;
    //             CRSF::GetMspMessage(&mspData, &mspLen);
    //             // if we have a new msp package start sending
    //             if (mspData != nullptr)
    //             {
    //                 MspSender.SetDataToTransmit(mspData, mspLen);
    //                 mspTransferActive = true;
    //             }
    //         }
    //     }
}
