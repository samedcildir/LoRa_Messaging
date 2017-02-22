#ifndef LORA_HPP
#define LORA_HPP

#include <stdint.h>

// SPI LoRa RELATED DEFINITIONS //
// NOTE: The FIFO data buffer location to be read from, or written to, via the SPI interface is defined by the address pointer
//RegFifoAddrPtr. Before any read or write operation it is hence necessary to initialize this pointer to the corresponding base
//value. Upon reading or writing to the FIFO data buffer (RegFifo) the address pointer will then increment automatically
#define RegFifo_ADR       0x00
#define RegOpMode_ADR     0x01
#define RegFrfMsb_ADR     0x06
#define RegFrfMid_ADR     0x07
#define RegFrfLsb_ADR     0x08
#define RegPaConfig_ADR   0x09
#define RegPaRamp_ADR     0x0A
#define RegOcp_ADR        0x0B
#define RegLna_ADR        0x0C
#define RegFifoAddrPtr_ADR      0x0D              // NOTE: set this and then send 0x00 to access this
#define RegFifoTxBaseAddr_ADR   0x0E              // NOTE: deafult -> 0x00
#define RegFifoRxBaseAddr_ADR   0x0F              // NOTE: deafult -> 0x80
#define FifoRxCurrentAddr_ADR   0x10              // NOTE: Start address (in data buffer) of last packet received
#define RegIrqFlagsMask_ADR     0x11              // NOTE: set to 0b10010111 // 1 -> off, 0 -> on
#define RegIrqFlags_ADR         0x12              // NOTE: 7 rxTimeout, 6 rxDone, 5 PayloadCRCError, 4 ValidHeader, 3 TxDone, 2 CadDone, 1 FhssChangeChannel, 0 CadDetected
#define RegRxNbBytes_ADR        0x13              // NOTE: Number of payload bytes of latest packet received
#define RegRxHeaderCntValueMsb_ADR    0x14
#define RegRxHeaderCntValueLsb_ADR    0x15
#define RegRxPacketCntValueMsb_ADR    0x16
#define RegRxPacketCntValueLsb_ADR    0x17
#define RegModemStat_ADR              0x18
#define RegPktSnrValue_ADR            0x19
#define RegPktRssiValue_ADR           0x1a
#define RegRssiValue_ADR              0x1b
#define RegHopChannel_ADR             0x1c
#define RegModemConfig1_ADR           0x1d        // NOTE: Signal bandwidth on 7-4 bits // default 125 kHz
#define RegModemConfig2_ADR           0x1e        // NOTE: SpreadingFactor on 7-4 bits  // default 128 chips / symbol // CRC On/Off setting on
#define RegSymbTimeoutLsb_ADR         0x1f
#define RegPreambleMsb_ADR            0x20
#define RegPreambleLsb_ADR            0x21
#define RegPayloadLength_ADR          0x22        // NOTE: transmit size
#define RegMaxPayloadLength_ADR       0x23
#define RegHopPeriod_ADR              0x24
#define RegFifoRxByteAddr_ADR         0x25        // NOTE: Current value of RX databuffer pointer (address of last byte written by Lora receiver)
#define RegModemConfig3_ADR           0x26
#define RegFeiMsb_ADR           0x28
#define RegFeiMid_ADR           0x29
#define RegFeiLsb_ADR           0x2a
#define RegRssiWideband_ADR     0x2c
#define RegDetectOptimize_ADR   0x31              // NOTE: SF6 olursa değişicek sanırım // default SF7
#define RegInvertIQ_ADR         0x33
#define RegDetectionThreshold_ADR     0x37        // NOTE: SF6 olursa değişicek sanırım // default SF7
#define RegSyncWord_ADR         0x39              // NOTE: LoRa Sync Word // Value 0x34 is reserved for LoRaWAN networks // default 0x12
#define RegDioMapping1_ADR      0x40
#define RegDioMapping2_ADR      0x41
#define RegVersion_ADR          0x42
#define RegTcxo_ADR             0x4b
#define RegPaDac_ADR            0x4d
#define RegFormerTemp_ADR       0x5b
#define RegAgcRef_ADR           0x61
#define RegAgcThresh1_ADR       0x62
#define RegAgcThresh2_ADR       0x63
#define RegAgcThresh3_ADR       0x64
#define RegPll_ADR              0x70

void init_lora();
void lora_cont_recv();
void lora_send(uint8_t sz, uint8_t *data);

#endif
