/* ELRS include */

#include "rxtx_common.h"

// #include "CRSFHandset.h"
// #include "dynpower.h"
// #include "lua.h"
#include "msp.h"
#include "msptypes.h"
#include "telemetry_protocol.h"
#include "stubborn_receiver.h"
#include "stubborn_sender.h"

// #include "devHandset.h"
// #include "devLED.h"
// #include "devScreen.h"
// #include "devBuzzer.h"
// #include "devBLE.h"
// #include "devLUA.h"
// #include "devWIFI.h"
// #include "devButton.h"
// #include "devVTX.h"
// #include "devGsensor.h"
// #include "devThermal.h"
// #include "devPDET.h"
// #include "devBackpack.h"

// #include "MAVLink.h"

/* User include */

#include <Adafruit_SSD1306.h>
#include "TimerInterrupt_Generic.h"

/* ELRS variable */

// #define MSP_PACKET_SEND_INTERVAL 10LU

// MSP msp;
ELRS_EEPROM eeprom;

FIFO<AP_MAX_BUF_LEN> apInputBuffer;
FIFO<AP_MAX_BUF_LEN> apOutputBuffer;

// #define UART_INPUT_BUF_LEN 1024
// FIFO<UART_INPUT_BUF_LEN> uartInputBuffer;

// // Buffer for current stubbon sender packet (mavlink only)
// uint8_t mavlinkSSBuffer[CRSF_MAX_PACKET_LEN]; 

// bool NextPacketIsMspData = false;  

// #define syncSpamAmount 3
// #define syncSpamAmountAfterRateChange 10
// volatile uint8_t syncSpamCounter = 0;
// volatile uint8_t syncSpamCounterAfterRateChange = 0;
uint32_t rfModeLastChangedMS = 0;
uint32_t SyncPacketLastSent = 0;


volatile uint32_t LastTLMpacketRecvMillis = 0;
// uint32_t TLMpacketReported = 0;
// static bool commitInProgress = false;

LQCALC<25> LQCalc;

volatile bool busyTransmitting;
// static volatile bool ModelUpdatePending;

uint8_t MSPDataPackage[5];
#define BindingSpamAmount 25
static uint8_t BindingSendCount;
// bool RxWiFiReadyToSend = false;

// bool headTrackingEnabled = false;
// #if !defined(CRITICAL_FLASH)
// static uint16_t ptrChannelData[3] = {CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MID, CRSF_CHANNEL_VALUE_MID};
// static uint32_t lastPTRValidTimeMs;
// #endif

static TxTlmRcvPhase_e TelemetryRcvPhase = ttrpTransmitting;
StubbornReceiver TelemetryReceiver;
StubbornSender MspSender;
// uint8_t CRSFinBuffer[CRSF_MAX_PACKET_LEN+1];

/* User variable */

#define OLED_RESET 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(OLED_RESET);

#define AIRRATE RATE_LORA_333HZ_8CH
#define BUADRATE 14400
bool isAirPort = false;

uint16_t fullScount;
uint16_t fullSfreq;
uint16_t fullRcount;
uint16_t fullRfreq;
uint16_t validSendCount;
uint16_t validSendFreq;
uint16_t validReceiveCount;
uint16_t validReceiveFreq;

#define TIMER_INTERVAL_MS 100000
STM32Timer ITimer(TIM2);

uint8_t CRCvalue;

#define PACKETSIZE 255
uint8_t StubbornSenderBuffer[PACKETSIZE];
uint8_t StubbornReceiverBuffer[PACKETSIZE];

/* ELRS Function*/

// void ICACHE_RAM_ATTR LinkStatsFromOta(OTA_LinkStats_s * const ls) // ELRS移植，注释源码另起修改
// {
//   int8_t snrScaled = ls->SNR;
//   DynamicPower_TelemetryUpdate(snrScaled);

//   // Antenna is the high bit in the RSSI_1 value
//   // RSSI received is signed, inverted polarity (positive value = -dBm)
//   // OpenTX's value is signed and will display +dBm and -dBm properly
//   CRSF::LinkStatistics.uplink_RSSI_1 = -(ls->uplink_RSSI_1);
//   CRSF::LinkStatistics.uplink_RSSI_2 = -(ls->uplink_RSSI_2);
//   CRSF::LinkStatistics.uplink_Link_quality = ls->lq;
// #if defined(DEBUG_FREQ_CORRECTION)
//   // Don't descale the FreqCorrection value being send in SNR
//   CRSF::LinkStatistics.uplink_SNR = snrScaled;
// #else
//   CRSF::LinkStatistics.uplink_SNR = SNR_DESCALE(snrScaled);
// #endif
//   CRSF::LinkStatistics.active_antenna = ls->antenna;
//   connectionHasModelMatch = ls->modelMatch;
//   -- downlink_SNR / downlink_RSSI is updated for any packet received, not just Linkstats
//   -- uplink_TX_Power is updated when sending to the handset, so it updates when missing telemetry
//   -- rf_mode is updated when we change rates
//   -- downlink_Link_quality is updated before the LQ period is incremented
//   MspSender.ConfirmCurrentPayload(ls->mspConfirm);
// }

bool ICACHE_RAM_ATTR ProcessTLMpacket(SX12xxDriverCommon::rx_status const status) // ELRS移植，注释源码另起修改
{
  if (status != SX12xxDriverCommon::SX12XX_RX_OK)
  {
    // Serial.println("TLM HW CRC error");
    // DBGLN("TLM HW CRC error");
    return false;
  }

  OTA_Packet_s * const otaPktPtr = (OTA_Packet_s * const)Radio.RXdataBuffer;
  if (!OtaValidatePacketCrc(otaPktPtr))
  {    
    // Serial.println("TLM crc error");
    // DBGLN("TLM crc error");
    return false;
  }

  if (otaPktPtr->std.type != PACKET_TYPE_TLM)
  {    
    // Serial.println("TLM type error " + String(otaPktPtr->std.type));
    // DBGLN("TLM type error %d", otaPktPtr->std.type);
    return false;
  }

  LastTLMpacketRecvMillis = millis();
  LQCalc.add();
  //
  // Radio.GetLastPacketStats();
  // CRSF::LinkStatistics.downlink_SNR = SNR_DESCALE(Radio.LastPacketSNRRaw);
  // CRSF::LinkStatistics.downlink_RSSI_1 = Radio.LastPacketRSSI;
  // CRSF::LinkStatistics.downlink_RSSI_2 = Radio.LastPacketRSSI2;

  // Full res mode
  if (OtaIsFullRes)
  {
    OTA_Packet8_s * const ota8 = (OTA_Packet8_s * const)otaPktPtr;
    uint8_t *telemPtr;
    uint8_t dataLen;
    // if (ota8->tlm_dl.containsLinkStats)
    // {
    //   LinkStatsFromOta(&ota8->tlm_dl.ul_link_stats.stats);
    //   telemPtr = ota8->tlm_dl.ul_link_stats.payload;
    //   dataLen = sizeof(ota8->tlm_dl.ul_link_stats.payload);
    // }
    // else
    {
      if(isAirPort && otaPktPtr->full.airport.count)
      // if (firmwareOptions.is_airport)
      {
        validReceiveCount++;
        OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
        return true;
      }
      MspSender.ConfirmCurrentPayload(ota8->tlm_dl.containsLinkStats);
      telemPtr = ota8->tlm_dl.payload;
      dataLen = sizeof(ota8->tlm_dl.payload);
    }
    // //DBGLN("pi=%u len=%u", ota8->tlm_dl.packageIndex, dataLen);
    validReceiveCount++;
    TelemetryReceiver.ReceiveData(ota8->tlm_dl.packageIndex & ELRS8_TELEMETRY_MAX_PACKAGES, telemPtr, dataLen);
  }
  // // Std res mode
  // else
  // {
  //   switch (otaPktPtr->std.tlm_dl.type)
  //   {
  //     case ELRS_TELEMETRY_TYPE_LINK:
  //       LinkStatsFromOta(&otaPktPtr->std.tlm_dl.ul_link_stats.stats);
  //       break;

  //     case ELRS_TELEMETRY_TYPE_DATA:
  //       if(otaPktPtr->std.airport.count)
  //       // if (firmwareOptions.is_airport)
  //       {
  //         validReceiveCount++;
  //         OtaUnpackAirportData(otaPktPtr, &apOutputBuffer);
  //         return true;
  //       }
  //       // TelemetryReceiver.ReceiveData(otaPktPtr->std.tlm_dl.packageIndex & ELRS4_TELEMETRY_MAX_PACKAGES,
  //       //   otaPktPtr->std.tlm_dl.payload,
  //       //   sizeof(otaPktPtr->std.tlm_dl.payload));
  //       break;
  //   }
  // }

  return true;
}

void ICACHE_RAM_ATTR GenerateSyncPacketData(OTA_Sync_s * const syncPtr) // ELRS移植，注释源码另起修改
{
  // const uint8_t SwitchEncMode = config.GetSwitchMode();
  // const uint8_t Index = (syncSpamCounter) ? config.GetRate() : ExpressLRS_currAirRate_Modparams->index;

  // if (syncSpamCounter)
  //   --syncSpamCounter;

  // if (syncSpamCounterAfterRateChange && Index == ExpressLRS_currAirRate_Modparams->index)
  // {
  //   --syncSpamCounterAfterRateChange;
  //   if (connectionState == connected) // We are connected again after a rate change.  No need to keep spaming sync.
  //     syncSpamCounterAfterRateChange = 0;
  // }

  SyncPacketLastSent = millis();

  // expresslrs_tlm_ratio_e newTlmRatio = UpdateTlmRatioEffective();

  syncPtr->fhssIndex = FHSSgetCurrIndex();
  syncPtr->nonce = OtaNonce;
  // syncPtr->rateIndex = Index;
  // syncPtr->newTlmRatio = newTlmRatio - TLM_RATIO_NO_TLM;
  // syncPtr->switchEncMode = SwitchEncMode;
  syncPtr->UID3 = UID[3];
  syncPtr->UID4 = UID[4];
  syncPtr->UID5 = UID[5];

  // // For model match, the last byte of the binding ID is XORed with the inverse of the modelId
  // if (!InBindingMode && config.GetModelMatch())
  // {
  //   syncPtr->UID5 ^= (~CRSFHandset::getModelID()) & MODELMATCH_MASK;
  // }
}

void SetRFLinkRate(uint8_t index) // ELRS移植，注释源码另起修改
{
    expresslrs_mod_settings_s *const ModParams = get_elrs_airRateConfig(index);
    expresslrs_rf_pref_params_s *const RFperf = get_elrs_RFperfParams(index);
    // Binding always uses invertIQ
    bool invertIQ = InBindingMode || (UID[5] & 0x01);
    // OtaSwitchMode_e newSwitchMode = (OtaSwitchMode_e)config.GetSwitchMode();

  if ((ModParams == ExpressLRS_currAirRate_Modparams)
    && (RFperf == ExpressLRS_currAirRate_RFperfParams)
    && (invertIQ == Radio.IQinverted)
    // && (OtaSwitchModeCurrent == newSwitchMode)
    )
    return;
    Serial.println("set rate " + String(index));
    // DBGLN("set rate %u", index);
    uint32_t interval = ModParams->interval;
#if defined(DEBUG_FREQ_CORRECTION) && defined(RADIO_SX128X)
  interval = interval * 12 / 10; // increase the packet interval by 20% to allow adding packet header
#endif
  hwTimer::updateInterval(interval);

    FHSSusePrimaryFreqBand = !(ModParams->radio_type == RADIO_TYPE_LR1121_LORA_2G4) && !(ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4);
    FHSSuseDualBand = ModParams->radio_type == RADIO_TYPE_LR1121_LORA_DUAL;

  Radio.Config(ModParams->bw, ModParams->sf, ModParams->cr, FHSSgetInitialFreq(),
               ModParams->PreambleLen, invertIQ, ModParams->PayloadLength, ModParams->interval
#if defined(RADIO_SX128X)
               , uidMacSeedGet(), OtaCrcInitializer, (ModParams->radio_type == RADIO_TYPE_SX128x_FLRC)
#endif
#if defined(RADIO_LR1121)
               , (ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 || ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4), (uint8_t)UID[5], (uint8_t)UID[4]
#endif
               );

#if defined(RADIO_LR1121)
  if (FHSSuseDualBand)
  {
    Radio.Config(ModParams->bw2, ModParams->sf2, ModParams->cr2, FHSSgetInitialGeminiFreq(),
                ModParams->PreambleLen2, invertIQ, ModParams->PayloadLength, ModParams->interval,
                (ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_900 || ModParams->radio_type == RADIO_TYPE_LR1121_GFSK_2G4),
                (uint8_t)UID[5], (uint8_t)UID[4], SX12XX_Radio_2);
  }
#endif

    Radio.FuzzySNRThreshold = (RFperf->DynpowerSnrThreshUp == DYNPOWER_SNR_THRESH_NONE) ? 0 : (RFperf->DynpowerSnrThreshUp - RFperf->DynpowerSnrThreshDn);

    // if ((isDualRadio() && config.GetAntennaMode() == TX_RADIO_MODE_GEMINI) || FHSSuseDualBand) // Gemini mode
    // {
    //     Radio.SetFrequencyReg(FHSSgetInitialGeminiFreq(), SX12XX_Radio_2);
    // }

    // InitialFreq has been set, so lets also reset the FHSS Idx and Nonce.
    FHSSsetCurrIndex(0);
    OtaNonce = 0;

    OtaUpdateSerializers(smWideOr8ch, ModParams->PayloadLength);
    // OtaUpdateSerializers(newSwitchMode, ModParams->PayloadLength);
    MspSender.setMaxPackageIndex(31);
    // MspSender.setMaxPackageIndex(ELRS_MSP_MAX_PACKAGES);
    TelemetryReceiver.setMaxPackageIndex(OtaIsFullRes ? ELRS8_TELEMETRY_MAX_PACKAGES : ELRS4_TELEMETRY_MAX_PACKAGES);

    ExpressLRS_currAirRate_Modparams = ModParams;
    ExpressLRS_currAirRate_RFperfParams = RFperf;
    // CRSF::LinkStatistics.rf_Mode = ModParams->enum_rate;

    // handset->setPacketInterval(interval * ExpressLRS_currAirRate_Modparams->numOfSends);
    connectionState = disconnected;
    rfModeLastChangedMS = millis();
}

void ICACHE_RAM_ATTR HandleFHSS() // ELRS移植，注释源码另起修改
{
  uint8_t modresult = (OtaNonce + 1) % ExpressLRS_currAirRate_Modparams->FHSShopInterval;
  // If the next packet should be on the next FHSS frequency, do the hop
  if (!InBindingMode && modresult == 0)
  {
    // // Gemini mode
    // // If using DualBand always set the correct frequency band to the radios.  The HighFreq/LowFreq Tx amp is set during config.
    // if ((isDualRadio() && config.GetAntennaMode() == TX_RADIO_MODE_GEMINI) || FHSSuseDualBand)
    // {
    //     // Optimises the SPI traffic order.
    //     if (Radio.GetProcessingPacketRadio() == SX12XX_Radio_1)
    //     {
    //         uint32_t freqRadio = FHSSgetNextFreq();
    //         Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2);
    //         Radio.SetFrequencyReg(freqRadio, SX12XX_Radio_1);
    //     }
    //     else
    //     {
    //         Radio.SetFrequencyReg(FHSSgetNextFreq(), SX12XX_Radio_1);
    //         Radio.SetFrequencyReg(FHSSgetGeminiFreq(), SX12XX_Radio_2);
    //     }
    // }
    // else
    {
      Radio.SetFrequencyReg(FHSSgetNextFreq());
    }
  }
}

void ICACHE_RAM_ATTR HandlePrepareForTLM() // ELRS移植，注释源码另起修改
{
  // If TLM enabled and next packet is going to be telemetry, start listening to have a large receive window (time-wise)
  if (ExpressLRS_currTlmDenom != 1 && ((OtaNonce + 1) % ExpressLRS_currTlmDenom) == 0)
  {
    Radio.RXnb();
    TelemetryRcvPhase = ttrpPreReceiveGap;
  }
}

void ICACHE_RAM_ATTR SendRCdataToRF() // ELRS移植，注释源码另起修改
{
  // // Do not send a stale channels packet to the RX if one has not been received from the handset
  // // *Do* send data if a packet has never been received from handset and the timer is running
  // // this is the case when bench testing and TXing without a handset
  // bool dontSendChannelData = false;
  // uint32_t lastRcData = handset->GetRCdataLastRecv();
  // if (lastRcData && (micros() - lastRcData > 1000000))
  // {
  //   // The tx is in Mavlink mode and without a valid crsf or RC input.  Do not send stale or fake zero packet RC!
  //   // Only send sync and MSP packets.
  //   if (config.GetLinkMode() == TX_MAVLINK_MODE)
  //   {
  //     dontSendChannelData = true;
  //   }
  //   else
  //   {
  //     return;
  //   }
  // }

  busyTransmitting = true;

  // uint32_t const now = millis();
  // ESP requires word aligned buffer
  WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
//   static uint8_t syncSlot;
//
//   const bool isTlmDisarmed = config.GetTlm() == TLM_RATIO_DISARMED;
//   uint32_t SyncInterval = (connectionState == connected && !isTlmDisarmed) ? ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalConnected : ExpressLRS_currAirRate_RFperfParams->SyncPktIntervalDisconnected;
//   bool skipSync = InBindingMode ||
//     // TLM_RATIO_DISARMED keeps sending sync packets even when armed until the RX stops sending telemetry and the TLM=Off has taken effect
//     (isTlmDisarmed && handset->IsArmed() && (ExpressLRS_currTlmDenom == 1));
//
  uint8_t NonceFHSSresult = OtaNonce % ExpressLRS_currAirRate_Modparams->FHSShopInterval;

  // Sync spam only happens on slot 1 and 2 and can't be disabled
  if (!InBindingMode && FHSSonSyncChannel())
  // if ((syncSpamCounter || (syncSpamCounterAfterRateChange && FHSSonSyncChannel())) && (NonceFHSSresult == 1 || NonceFHSSresult == 2))
  {
    otaPkt.std.type = PACKET_TYPE_SYNC;
    GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
    // syncSlot = 0; // reset the sync slot in case the new rate (after the syncspam) has a lower FHSShopInterval
  }
  // // Regular sync rotates through 4x slots, twice on each slot, and telemetry pushes it to the next slot up
  // // But only on the sync FHSS channel and with a timed delay between them
  // else if ((!skipSync) && ((syncSlot / 2) <= NonceFHSSresult) && (now - SyncPacketLastSent > SyncInterval) && FHSSonSyncChannel())
  // {
  //   otaPkt.std.type = PACKET_TYPE_SYNC;
  //   GenerateSyncPacketData(OtaIsFullRes ? &otaPkt.full.sync.sync : &otaPkt.std.sync);
  //   syncSlot = (syncSlot + 1) % (ExpressLRS_currAirRate_Modparams->FHSShopInterval * 2);
  // }
  else
  {
    if(isAirPort && apInputBuffer.size())
    {
      OtaPackAirportData(&otaPkt, &apInputBuffer);
    }
    else if (MspSender.IsActive())
    // if ((NextPacketIsMspData && MspSender.IsActive()) || dontSendChannelData)
    {
      otaPkt.std.type = PACKET_TYPE_MSPDATA;
      if (OtaIsFullRes)
      {
        otaPkt.full.msp_ul.packageIndex = MspSender.GetCurrentPayload(
          otaPkt.full.msp_ul.payload,
          sizeof(otaPkt.full.msp_ul.payload));
        // if (config.GetLinkMode() == TX_MAVLINK_MODE)
          otaPkt.full.msp_ul.tlmFlag = TelemetryReceiver.GetCurrentConfirm();
      }
      // else
      // {
      //   otaPkt.std.msp_ul.packageIndex = MspSender.GetCurrentPayload(
      //     otaPkt.std.msp_ul.payload,
      //     sizeof(otaPkt.std.msp_ul.payload));
      //   if (config.GetLinkMode() == TX_MAVLINK_MODE)
      //     otaPkt.std.msp_ul.tlmFlag = TelemetryReceiver.GetCurrentConfirm();
      // }
      validSendCount++;
      // // send channel data next so the channel messages also get sent during msp transmissions
      // NextPacketIsMspData = false;
      // counter can be increased even for normal msp messages since it's reset if a real bind message should be sent
      BindingSendCount++;
      // // If the telemetry ratio isn't already 1:2, send a sync packet to boost it
      // // to add bandwidth for the reply
      // if (ExpressLRS_currTlmDenom != 2)
      //   syncSpamCounter = 1;
    }
    else
    {
      // // always enable msp after a channel package since the slot is only used if MspSender has data to send
      // NextPacketIsMspData = true;
      otaPkt.std.type = PACKET_TYPE_MSPDATA;
      otaPkt.full.msp_ul.tlmFlag = TelemetryReceiver.GetCurrentConfirm();
      // injectBackpackPanTiltRollData(now);
      // OtaPackChannelData(&otaPkt, ChannelData, TelemetryReceiver.GetCurrentConfirm(), ExpressLRS_currTlmDenom);
    }
  }

  ///// Next, Calculate the CRC and put it into the buffer /////
  OtaGeneratePacketCrc(&otaPkt);

  if(OtaIsFullRes)
    CRCvalue = otaPkt.full.crc;
  else 
    CRCvalue = otaPkt.std.crcLow | otaPkt.std.crcHigh;
    
  SX12XX_Radio_Number_t transmittingRadio = Radio.GetLastSuccessfulPacketRadio();

  // if (isDualRadio())
  // {
  //   switch (config.GetAntennaMode())
  //   {
  //   case TX_RADIO_MODE_GEMINI:
  //     transmittingRadio = SX12XX_Radio_All; // Gemini mode
  //     break;
  //   case TX_RADIO_MODE_ANT_1:
  //     transmittingRadio = SX12XX_Radio_1; // Single antenna tx and true diversity rx for tlm receiption.
  //     break;
  //   case TX_RADIO_MODE_ANT_2:
  //     transmittingRadio = SX12XX_Radio_2; // Single antenna tx and true diversity rx for tlm receiption.
  //     break;
  //   case TX_RADIO_MODE_SWITCH:
  //     if(OtaNonce%2==0)   transmittingRadio = SX12XX_Radio_1; // Single antenna tx and true diversity rx for tlm receiption.
  //     else   transmittingRadio = SX12XX_Radio_2; // Single antenna tx and true diversity rx for tlm receiption.
  //     break;
  //   default:
  //     break;
  //   }
  // }

#if defined(Regulatory_Domain_EU_CE_2400)
  transmittingRadio &= ChannelIsClear(transmittingRadio);   // weed out the radio(s) if channel in use

  if (transmittingRadio == SX12XX_Radio_NONE)
  {
    // No packet will be sent due to LBT.
    // Defer TXdoneCallback() to prepare for TLM when the IRQ is normally triggered.
    deferExecutionMicros(ExpressLRS_currAirRate_RFperfParams->TOA, Radio.TXdoneCallback);
  }
  else
#endif
  {
    Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength, transmittingRadio);
  }
}

void ICACHE_RAM_ATTR HWtimerCallbackTock() // ELRS移植，注释源码另起修改
{
  // /* If we are busy writing to EEPROM (committing config changes) then we just advance the nonces, i.e. no SPI traffic */
  // if (commitInProgress)
  // {
  //   nonceAdvance();
  //   return;
  // }

  // Radio.isFirstRxIrq = true;

  // // Sync OpenTX to this point
  // if (!(OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends))
  // {
  //   handset->JustSentRFpacket();
  // }

  // // Do not transmit or advance FHSS/Nonce until in disconnected/connected state
  // if (connectionState == awaitingModelId)
  //   return;

  // // Tx Antenna Diversity
  // if ((OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends == 0 || // Swicth with new packet data
  //     OtaNonce % ExpressLRS_currAirRate_Modparams->numOfSends == ExpressLRS_currAirRate_Modparams->numOfSends / 2) && // Swicth in the middle of DVDA sends
  //     TelemetryRcvPhase == ttrpTransmitting) // Only switch when transmitting.  A diversity rx will send tlm back on the best antenna.  So dont switch away from it.
  // {
  //   switchDiversityAntennas();
  // }

  // Nonce advances on every timer tick
  if (!InBindingMode)
    OtaNonce++;

  // If HandleTLM has started Receive mode, TLM packet reception should begin shortly
  // Skip transmitting on this slot
  if (TelemetryRcvPhase == ttrpPreReceiveGap)
  {
    TelemetryRcvPhase = ttrpExpectingTelem;
#if defined(Regulatory_Domain_EU_CE_2400)
    // Use downlink LQ for LBT success ratio instead for EU/CE reg domain
    CRSF::LinkStatistics.downlink_Link_quality = LBTSuccessCalc.getLQ();
#else
    // CRSF::LinkStatistics.downlink_Link_quality = LQCalc.getLQ();
#endif
    LQCalc.inc();
    return;
  }
  // else if (TelemetryRcvPhase == ttrpExpectingTelem && !LQCalc.currentIsSet())
  // {
  //   // // Indicate no telemetry packet received to the DP system
  //   // DynamicPower_TelemetryUpdate(DYNPOWER_UPDATE_MISSED);
  // }

  TelemetryRcvPhase = ttrpTransmitting;

  SendRCdataToRF();
}

bool ICACHE_RAM_ATTR RXdoneISR(SX12xxDriverCommon::rx_status const status) // ELRS移植，注释源码另起修改
{
  fullRcount++;

  if (LQCalc.currentIsSet())
  {
    return false; // Already received tlm, do not run ProcessTLMpacket() again.
  }

  bool packetSuccessful = ProcessTLMpacket(status);
#if defined(Regulatory_Domain_EU_CE_2400)
  if (packetSuccessful)
  {
    SetClearChannelAssessmentTime();
  }
#endif
  busyTransmitting = false;
  return packetSuccessful;
}

void ICACHE_RAM_ATTR TXdoneISR() // ELRS移植，注释源码另起修改
{
  if (!busyTransmitting)
  {
    return; // Already finished transmission and do not call HandleFHSS() a second time, which may hop the frequency!
  }
  fullScount++;
  if (connectionState != awaitingModelId)
  {
    HandleFHSS();
    HandlePrepareForTLM();
#if defined(Regulatory_Domain_EU_CE_2400)
    if (TelemetryRcvPhase != ttrpPreReceiveGap)
    {
      // Start RX for Listen Before Talk early because it takes about 100us
      // from RX enable to valid instant RSSI values are returned.
      // If rx was already started by TLM prepare above, this call will let RX
      // continue as normal.
      SetClearChannelAssessmentTime();
    }
#endif // non-CE
  }
  busyTransmitting = false;
}

static void UpdateConnectDisconnectStatus() // ELRS移植，注释源码另起修改
{
  // Number of telemetry packets which can be lost in a row before going to disconnected state
  constexpr unsigned RX_LOSS_CNT = 5;
  // Must be at least 512ms and +2 to account for any rounding down and partial millis()
  const uint32_t msConnectionLostTimeout = std::max((uint32_t)512U,
    (uint32_t)ExpressLRS_currTlmDenom * ExpressLRS_currAirRate_Modparams->interval / (1000U / RX_LOSS_CNT)
    ) + 2U;
  // Capture the last before now so it will always be <= now
  const uint32_t lastTlmMillis = LastTLMpacketRecvMillis;
  const uint32_t now = millis();
  if (lastTlmMillis && ((now - lastTlmMillis) <= msConnectionLostTimeout))
  {
    if (connectionState != connected)
    {
      connectionState = connected;
      // CRSFHandset::ForwardDevicePings = true;
      Serial.println("got downlink conn");
      // DBGLN("got downlink conn");

      apInputBuffer.flush();
      apOutputBuffer.flush();
      // uartInputBuffer.flush();

      // VtxTriggerSend();
    }
  }
  // If past RX_LOSS_CNT, or in awaitingModelId state for longer than DisconnectTimeoutMs, go to disconnected
  else if (connectionState == connected ||
    (now - rfModeLastChangedMS) > ExpressLRS_currAirRate_RFperfParams->DisconnectTimeoutMs)
  {
    connectionState = disconnected;
    // connectionHasModelMatch = true;
    // CRSFHandset::ForwardDevicePings = false;
  }
}

void SendUIDOverMSP() // ELRS移植，注释源码另起修改
{
  MSPDataPackage[0] = MSP_ELRS_BIND;
  memcpy(&MSPDataPackage[1], &UID[2], 4);
  BindingSendCount = 0;
  MspSender.ResetState();
  MspSender.SetDataToTransmit(MSPDataPackage, 5);
}

static void EnterBindingMode() // ELRS移植，注释源码另起修改
{
  if (InBindingMode)
    return;

  // Disable the TX timer and wait for any TX to complete
  hwTimer::stop();
  // while (busyTransmitting);

  // Queue up sending the Master UID as MSP packets
  SendUIDOverMSP();

  // Binding uses a CRCInit=0, 50Hz, and InvertIQ
  OtaCrcInitializer = 0;
  OtaNonce = 0; // Lock the OtaNonce to prevent syncspam packets
  InBindingMode = true; // Set binding mode before SetRFLinkRate() for correct IQ

  // Start attempting to bind
  // Lock the RF rate and freq while binding
  SetRFLinkRate(enumRatetoIndex(RATE_BINDING));

  // Start transmitting again
  hwTimer::resume();
  
  Serial.println("Entered binding mode at freq = " + String(Radio.currFreq));
  // DBGLN("Entered binding mode at freq = %d", Radio.currFreq);
}

static void ExitBindingMode() // ELRS移植，注释源码另起修改
{
  if (!InBindingMode)
    return;

  MspSender.ResetState();

  // Reset CRCInit to UID-defined value
  OtaUpdateCrcInitFromUid();
  InBindingMode = false; // Clear binding mode before SetRFLinkRate() for correct IQ

  SetRFLinkRate(enumRatetoIndex(AIRRATE)); //return to original rate
  // SetRFLinkRate(config.GetRate()); 

  Serial.println("Exiting binding mode");
  // DBGLN("Exiting binding mode");
}

static void HandleUARTout() // ELRS移植，注释源码另起修改
{
  if(Serial.availableForWrite())
  // if (firmwareOptions.is_airport)
  {
    auto size = apOutputBuffer.size();
    if (size)
    {
      uint8_t buf[size];
      apOutputBuffer.lock();
      apOutputBuffer.popBytes(buf, size);
      apOutputBuffer.unlock();
      Serial.write(buf, size);
      // TxUSB->write(buf, size);
    }
  }
}

static void HandleUARTin() // ELRS移植，注释源码另起修改
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
  //   // Read from the USB serial port
  // if (TxUSB->available())
  // {
  //   if (firmwareOptions.is_airport)
  //   {
  //     auto size = std::min(apInputBuffer.free(), (uint16_t)TxUSB->available());
  //     if (size > 0)
  //     {
  //       uint8_t buf[size];
  //       TxUSB->readBytes(buf, size);
  //       apInputBuffer.lock();
  //       apInputBuffer.pushBytes(buf, size);
  //       apInputBuffer.unlock();
  //     }
  //   }
  //   else
  //   {
  //     auto size = std::min(uartInputBuffer.free(), (uint16_t)TxUSB->available());
  //     if (size > 0)
  //     {
  //       uint8_t buf[size];
  //       TxUSB->readBytes(buf, size);
  //       uartInputBuffer.lock();
  //       uartInputBuffer.pushBytes(buf, size);
  //       uartInputBuffer.unlock();

  //       // Lets check if the data is Mav and auto change LinkMode
  //       // Start the hwTimer since the user might be operating the module as a standalone unit without a handset.
  //       if (connectionState == noCrossfire)
  //       {
  //         if (isThisAMavPacket(buf, size))
  //         {
  //           config.SetLinkMode(TX_MAVLINK_MODE);
  //           UARTconnected();
  //         }
  //       }
  //     }
  //   }
  // }

  // // Read from the Backpack serial port
  // if (TxBackpack->available())
  // {
  //   auto size = std::min(uartInputBuffer.free(), (uint16_t)TxBackpack->available());
  //   if (size > 0)
  //   {
  //     uint8_t buf[size];
  //     TxBackpack->readBytes(buf, size);

  //     // If the TX is in Mavlink mode, push the bytes into the fifo buffer
  //     if (config.GetLinkMode() == TX_MAVLINK_MODE)
  //     {
  //       uartInputBuffer.lock();
  //       uartInputBuffer.pushBytes(buf, size);
  //       uartInputBuffer.unlock();

  //       // The tx is in Mavlink mode and receiving data from the Backpack.
  //       // Start the hwTimer since the user might be operating the module as a standalone unit without a handset.
  //       if (connectionState == noCrossfire)
  //       {
  //         if (isThisAMavPacket(buf, size))
  //         {
  //           UARTconnected();
  //         }
  //       }
  //     }

  //     // Try to parse any MSP packets from the Backpack
  //     ParseMSPData(buf, size);
  //   }
  // }
}

static void setupBindingFromConfig() // ELRS移植，注释源码另起修改
{
  // if (firmwareOptions.hasUID)
  // {
  //     memcpy(UID, firmwareOptions.uid, UID_LEN);
  // }
  // else
  {
#ifdef PLATFORM_ESP32
    esp_read_mac(UID, ESP_MAC_WIFI_STA);
#elif PLATFORM_STM32
    UID[0] = (uint8_t)HAL_GetUIDw0();
    UID[1] = (uint8_t)(HAL_GetUIDw0() >> 8);
    UID[2] = (uint8_t)HAL_GetUIDw1();
    UID[3] = (uint8_t)(HAL_GetUIDw1() >> 8);
    UID[4] = (uint8_t)HAL_GetUIDw2();
    UID[5] = (uint8_t)(HAL_GetUIDw2() >> 8);
#endif
  }

    Serial.print("UID ");
    for (int i = 0; i < sizeof(UID); i++)
      Serial.print(String(UID[i]) + " ");
    Serial.println();
  // DBGLN("UID=(%d, %d, %d, %d, %d, %d)",
  //   UID[0], UID[1], UID[2], UID[3], UID[4], UID[5]);

  OtaUpdateCrcInitFromUid();
}

/* User Function */

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
        // send freq
        display.setCursor(0, 16);
        display.println("Send");
        display.setCursor(30, 16);
        display.println(validSendFreq);
        // full Send freq
        display.setCursor(54, 16);
        display.println("FullS");
        display.setCursor(92, 16);
        display.println(fullSfreq); 
        // receive freq
        display.setCursor(0, 24);
        display.println("Recv");
        display.setCursor(30, 24);
        display.println(validReceiveFreq);  
        // // full Recv freq
        // display.setCursor(54, 24);
        // display.println("FullR");
        // display.setCursor(92, 24);
        // display.println(fullRfreq);
        display.setCursor(54, 24);     
        display.println("RSSI");     
        display.setCursor(84, 24);           
        display.println(Radio.GetRssiInst(SX12XX_Radio_All)); 
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
        EnterBindingMode();
    }

}

void TimerHandler()
{
  static uint16_t timercount = 0;
  if(timercount % (1000000 / TIMER_INTERVAL_MS) == 0)
  {
    validSendFreq = validSendCount;
    validSendCount = 0;
    validReceiveFreq = validReceiveCount;
    validReceiveCount = 0;
    fullSfreq = fullScount;
    fullScount = 0;
    fullRfreq = fullRcount;
    fullRcount = 0;

    // uint8_t buf[140] = {"HELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOWORLDHELLOW\r\n"};
    // apInputBuffer.pushBytes(buf, sizeof(buf));
  }
  timercount++;

  if(connectionState == connected)
  {
    digitalWrite(PC13, LOW);
  }
  else
  {
    digitalToggle(PC13);
  }
}

void setupBasicHardWare(void)
{
    // UART
    Serial.begin(BUADRATE);
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


/* setup and loop */

void setup()
{
  setupBasicHardWare();

  setupBindingFromConfig();
  FHSSrandomiseFHSSsequence(uidMacSeedGet());

  Radio.TXdoneCallback = &TXdoneISR;
  Radio.RXdoneCallback = &RXdoneISR;

  Radio.currFreq = FHSSgetInitialFreq();

  Radio.Begin(FHSSgetMinimumFreq(), FHSSgetMaximumFreq());

  TelemetryReceiver.SetDataToReceive(StubbornReceiverBuffer, sizeof(StubbornReceiverBuffer));
  // TelemetryReceiver.SetDataToReceive(CRSFinBuffer, sizeof(CRSFinBuffer));

  SetRFLinkRate(enumRatetoIndex(AIRRATE));

  hwTimer::init(nullptr, HWtimerCallbackTock);
  hwTimer::resume();

  ExpressLRS_currTlmDenom = 2;

  Radio.SetOutputPower(SX1280_POWER_MAX);
}

void loop()
{
  uint32_t now = millis();

  // displayDebugInfo();

  HandleUARTout();

  HandleUARTin();

  if (connectionState < MODE_STATES)
  {
    UpdateConnectDisconnectStatus();
  }
  
  // only send msp data when binding is not active
  static bool mspTransferActive = false;

  if (TelemetryReceiver.HasFinishedData())
  {
      apOutputBuffer.lock();
      apOutputBuffer.atomicPushBytes(StubbornReceiverBuffer, PACKETSIZE);
      apOutputBuffer.unlock();
      TelemetryReceiver.Unlock();
  }
  if (InBindingMode)
  {
    // exit bind mode if package after some repeats
    if (BindingSendCount > BindingSpamAmount) 
    {
      ExitBindingMode();
    }
  }
  else if (!MspSender.IsActive())
  {
    auto size = apInputBuffer.size();
    if (size >= PACKETSIZE)
    {
      apInputBuffer.lock();
      apInputBuffer.popBytes(StubbornSenderBuffer, PACKETSIZE);
      apInputBuffer.unlock();
      MspSender.SetDataToTransmit(StubbornSenderBuffer, PACKETSIZE);
    }
  }
}
