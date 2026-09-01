#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief CAN filter configuration structure definition.
 */
typedef struct
{
    uint32_t FilterIdHigh;     /*!< Filter identification number MSBs for a 32-bit configuration,
                                    or the first ID for a 16-bit configuration.
                                    Range: 0x0000 to 0xFFFF. */
    uint32_t FilterIdLow;      /*!< Filter identification number LSBs for a 32-bit configuration,
                                    or the second ID for a 16-bit configuration.
                                    Range: 0x0000 to 0xFFFF. */
    uint32_t FilterMaskIdHigh; /*!< Filter mask or identification number MSBs for a 32-bit
                                   configuration, or the first value for a 16-bit configuration.
                                   Range: 0x0000 to 0xFFFF. */
    uint32_t FilterMaskIdLow;  /*!< Filter mask or identification number LSBs for a 32-bit
                                   configuration, or the second value for a 16-bit configuration.
                                   Range: 0x0000 to 0xFFFF. */
    uint32_t FilterFIFOAssignment; /*!< FIFO assigned to a matching filter: @ref CAN_FILTER_FIFO0
                                      or @ref CAN_FILTER_FIFO1. */
    uint32_t FilterBank;           /*!< Filter bank index. Range: 0 to 13 for a single CAN instance,
                                       or 0 to 27 for dual CAN instances. */
    uint32_t FilterMode;           /*!< Filter mode: identifier mask or identifier list;
                                       see @ref CAN_FILTERMODE_IDMASK and @ref CAN_FILTERMODE_IDLIST. */
    uint32_t FilterScale;          /*!< Filter scale: two 16-bit filters or one 32-bit filter;
                                       see @ref CAN_FILTERSCALE_16BIT and @ref CAN_FILTERSCALE_32BIT. */
    uint32_t FilterActivation;     /*!< Filter activation state: @ref CAN_FILTER_DISABLE
                                       or @ref CAN_FILTER_ENABLE. */
    uint32_t SlaveStartFilterBank; /*!< Start bank assigned to the slave CAN instance.
                                       Meaningless for a single CAN instance; for dual CAN,
                                       lower banks belong to the master and higher banks to the
                                       slave. Range: 0 to 27. */
} CAN_FilterTypeDef;

/**
 * @brief CAN Tx message header structure definition.
 */
typedef struct
{
    uint32_t        StdId; /*!< Standard identifier. Range: 0 to 0x7FF. */
    uint32_t        ExtId; /*!< Extended identifier. Range: 0 to 0x1FFFFFFF. */
    uint32_t        IDE;   /*!< Identifier type: @ref CAN_ID_STD or @ref CAN_ID_EXT. */
    uint32_t        RTR;   /*!< Frame type: @ref CAN_RTR_DATA or @ref CAN_RTR_REMOTE. */
    uint32_t        DLC;   /*!< Frame data length. Range: 0 to 8 bytes. */
    FunctionalState TransmitGlobalTime; /*!< Enable or disable transmission of the timestamp
                                            captured at the start of transmission. Time-triggered
                                            communication must be enabled; with DLC set to 8,
                                            the timestamp replaces DATA6 and DATA7. */
} CAN_TxHeaderTypeDef;

/**
 * @brief CAN Rx message header structure definition.
 */
typedef struct
{
    uint32_t StdId;            /*!< Standard identifier. Range: 0 to 0x7FF. */
    uint32_t ExtId;            /*!< Extended identifier. Range: 0 to 0x1FFFFFFF. */
    uint32_t IDE;              /*!< Identifier type: @ref CAN_ID_STD or @ref CAN_ID_EXT. */
    uint32_t RTR;              /*!< Frame type: @ref CAN_RTR_DATA or @ref CAN_RTR_REMOTE. */
    uint32_t DLC;              /*!< Frame data length. Range: 0 to 8 bytes. */
    uint32_t Timestamp;        /*!< Timestamp captured at the start of reception.
                                   Range: 0 to 0xFFFF. */
    uint32_t FilterMatchIndex; /*!< Index of the matching acceptance filter.
                                  Range: 0 to 0xFF. */
} CAN_RxHeaderTypeDef;

#define CAN_FILTERMODE_IDMASK (0x00000000U) /*!< Identifier mask mode. */
#define CAN_FILTERMODE_IDLIST (0x00000001U) /*!< Identifier list mode. */

#define CAN_FILTERSCALE_16BIT (0x00000000U) /*!< Two 16-bit filters. */
#define CAN_FILTERSCALE_32BIT (0x00000001U) /*!< One 32-bit filter. */

#define CAN_FILTER_DISABLE (0x00000000U) /*!< Disable filter. */
#define CAN_FILTER_ENABLE  (0x00000001U) /*!< Enable filter. */

#define CAN_FILTER_FIFO0 (0x00000000U) /*!< Assign matching frames to Rx FIFO 0. */
#define CAN_FILTER_FIFO1 (0x00000001U) /*!< Assign matching frames to Rx FIFO 1. */

#define CAN_ID_STD (0x00000000U) /*!< Standard identifier. */
#define CAN_ID_EXT (0x00000004U) /*!< Extended identifier. */

#define CAN_RTR_DATA   (0x00000000U) /*!< Data frame. */
#define CAN_RTR_REMOTE (0x00000002U) /*!< Remote frame. */

#ifdef __cplusplus
}
#endif
