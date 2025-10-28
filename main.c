/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define BAUD 45450     //  DELAY_TBIT = 44 // Working on Arduino @16MHz

//#define VCC_PIN     8
//#define GND_PIN     9
#define LED_PIN     GPIO_PIN_13
#define TOUCH_PIN   GPIO_PIN_0

unsigned long samplingtime = 0;

#define DELAY_TBIT      44  // This is timer delay for 1 TBIT
#define TX_ENABLE_PIN   GPIO_PIN_15   // This pin toggles the MAX485 IC from Transmit to Recieve
#define SLAVE_ADDRESS   4   // This is the address of this slave device
#define LED_ERROR_PIN   GPIO_PIN_14  // This is the built in led on pin 13 Arduino Uno - PB5

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
#define LED_ERROR_ON    HAL_GPIO_WritePin(GPIOC, LED_ERROR_PIN, GPIO_PIN_SET);
#define LED_ERROR_OFF   HAL_GPIO_WritePin(GPIOC, LED_ERROR_PIN, GPIO_PIN_RESET);

#define TX_ENABLE_ON    HAL_GPIO_WritePin(GPIOC, TX_ENABLE_PIN, GPIO_PIN_SET);
#define TX_ENABLE_OFF   HAL_GPIO_WritePin(GPIOC, TX_ENABLE_PIN, GPIO_PIN_RESET);
///////////////////////////////////////////////////////////////////////////////////////////////////
#define TIMER1_RUN  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
#define TIMER1_STOP HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
//////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Ident Nummer DP Slave. Arbitrarily chosen. This ID not found in my .GSD library.
///////////////////////////////////////////////////////////////////////////////////////////////////
#define IDENT_HIGH_BYTE       0xC0
#define IDENT_LOW_BYTE        0xDE
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
// Addresses
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SAP_OFFSET            128   // Service Access Point Adress Offset
#define BROADCAST_ADD         127
#define DEFAULT_ADD           126   // Delivery address
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Command types
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SD1                   0x10  // Telegram without data field
#define SD2                   0x68  // Data telegram variable
#define SD3                   0xA2  // Data telegram fixed
#define SD4                   0xDC  // Token
#define SC                    0xE5  // Short acknowledgment
#define ED                    0x16  // End
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Function Codes
///////////////////////////////////////////////////////////////////////////////////////////////////
/* FC Request */
#define FDL_STATUS            0x09  // SPS: Status query
#define SRD_HIGH              0x0D  // SPS: Set outputs, read inputs
#define FCV_                  0x10
#define FCB_                  0x20
#define REQUEST_              0x40

/* FC Response */
#define FDL_STATUS_OK         0x00  // SLA: OK
#define DATA_LOW              0x08  // SLA: (Data low) Send data inputs
#define DIAGNOSE              0x0A  // SLA: (Data high) Diagnosis pending
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Service Access Points (DP Slave) MS0
///////////////////////////////////////////////////////////////////////////////////////////////////
#define SAP_SET_SLAVE_ADR     55  // Master sets slave address, slave responds with SC
#define SAP_RD_INP            56  // Master requests input data, slave sends input data
#define SAP_RD_OUTP           57  // Master requests output data, slave sends output data
#define SAP_GLOBAL_CONTROL    58  // Master Control, Slave Does not answer
#define SAP_GET_CFG           59  // Master requests config., Slave sends configuration
#define SAP_SLAVE_DIAGNOSIS   60  // Master requests diagnosis, slave sends diagnosis Daten
#define SAP_SET_PRM           61  // Master sends parameters, slave sends SC
#define SAP_CHK_CFG           62  // Master sends configuration, Slave sends SC
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Global Control (Data Master)
///////////////////////////////////////////////////////////////////////////////////////////////////
#define CLEAR_DATA_           0x02
#define UNFREEZE_             0x04
#define FREEZE_               0x08
#define UNSYNC_               0x10
#define SYNC_                 0x20
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Diagnosis (Answer slave)
///////////////////////////////////////////////////////////////////////////////////////////////////
/* Status Byte 1 */
#define STATION_NOT_EXISTENT_ 0x01
#define STATION_NOT_READY_    0x02
#define CFG_FAULT_            0x04
#define EXT_DIAG_             0x08  // Extended diagnosis available
#define NOT_SUPPORTED_        0x10
#define INV_SLAVE_RESPONSE_   0x20
#define PRM_FAULT_            0x40
#define MASTER_LOCK           0x80

/* Status Byte 2 */
#define STATUS_2_DEFAULT      0x04
#define PRM_REQ_              0x01
#define STAT_DIAG_            0x02
#define WD_ON_                0x08
#define FREEZE_MODE_          0x10
#define SYNC_MODE_            0x20
#define DEACTIVATED_          0x80

/* Status Byte 3 */
#define DIAG_SIZE_OK          0x00
#define DIAG_SIZE_ERROR       0x80

/* Address */
#define MASTER_ADD_DEFAULT    0xFF

/* Extended diagnosis (EXT_DIAG_ = 1) */
#define EXT_DIAG_TYPE_        0xC0  // Bit 6-7 ist Diagnose Typ
#define EXT_DIAG_GERAET       0x00  // Wenn Bit 7 und 6 = 00, dann Geraetebezogen
#define EXT_DIAG_KENNUNG      0x40  // Wenn Bit 7 und 6 = 01, dann Kennungsbezogen
#define EXT_DIAG_KANAL        0x80  // Wenn Bit 7 und 6 = 10, dann Kanalbezogen

#define EXT_DIAG_BYTE_CNT_    0x3F  // Bit 0-5 sind Anzahl der Diagnose Bytes
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Set Parameters Request (Data master)
///////////////////////////////////////////////////////////////////////////////////////////////////
/* Command */
#define LOCK_SLAVE_           0x80  // Slave fuer andere Master gesperrt
#define UNLOCK_SLAVE_         0x40  // Slave fuer andere Master freigegeben
#define ACTIVATE_SYNC_        0x20
#define ACTIVATE_FREEZE_      0x10
#define ACTIVATE_WATCHDOG_    0x08

/* DPV1 Status Byte 1 */
#define DPV1_MODE_            0x80
#define FAIL_SAVE_MODE_       0x40
#define PUBLISHER_MODE_       0x20
#define WATCHDOG_TB_1MS       0x04

/* DPV1 Status Byte 2 */
#define PULL_PLUG_ALARM_      0x80
#define PROZESS_ALARM_        0x40
#define DIAGNOSE_ALARM_       0x20
#define VENDOR_ALARM_         0x10
#define STATUS_ALARM_         0x08
#define UPDATE_ALARM_         0x04
#define CHECK_CONFIG_MODE_    0x01

/* DPV1 Status Byte 3 */
#define PARAMETER_CMD_ON_     0x80
#define ISOCHRON_MODE_ON_     0x10
#define PARAMETER_BLOCK_      0x08
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// SAP: Check Config Request (Data Master)
///////////////////////////////////////////////////////////////////////////////////////////////////
#define CFG_DIRECTION_        0x30  // Bit 4-5 is direction. 01 = input, 10 = output, 11 = input / output
#define CFG_INPUT             0x10  // Input
#define CFG_OUTPUT            0x20  // Output
#define CFG_INPUT_OUTPUT      0x30  // Input/Output
#define CFG_SPECIAL           0x00  // Special format if more than 16/32 bytes are to be transferred

#define CFG_KONSISTENZ_       0x80  // Bit 7 is consistency. 0 = byte or word, 1 = over entire module
#define CFG_KONS_BYTE_WORT    0x00  // Byte or word
#define CFG_KONS_MODUL        0x80  // Modul

#define CFG_WIDTH_            0x40  // Bit 6 is IO width. 0 = byte (8bit), 1 = word (16bit)
#define CFG_BYTE              0x00  // Byte
#define CFG_WORD              0x40  // Word

/* Compact format */
#define CFG_BYTE_CNT_         0x0F  // Bit 0-3 are number of bytes or words. 0 = 1 byte, 1 = 2 bytes etc.

/* Special format */
#define CFG_SP_DIRECTION_     0xC0  // Bit 6-7 is direction. 01 = input, 10 = output, 11 = input / output
#define CFG_SP_VOID           0x00  // Empty space
#define CFG_SP_INPUT          0x40  // Input
#define CFG_SP_OUTPUT         0x80  // Output
#define CFG_SP_INPUT_OPTPUT   0xC0  // Input/Output

#define CFG_SP_VENDOR_CNT_    0x0F  // Bits 0-3 are the number of manufacturer-specific bytes. 0 = none

/* Special format / length byte */
#define CFG_SP_BYTE_CNT_      0x3F  // Bit 0-5 are number of bytes or words. 0 = 1 byte, 1 = 2 bytes etc.
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
#define TIMEOUT_MAX_SYN_TIME  33 * DELAY_TBIT // 33 TBit = TSYN
#define TIMEOUT_MAX_RX_TIME   15 * DELAY_TBIT
#define TIMEOUT_MAX_TX_TIME   15 * DELAY_TBIT
#define TIMEOUT_MAX_SDR_TIME  15 * DELAY_TBIT // 15 Tbit = TSDR
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
#define MAX_BUFFER_SIZE       32

#define INPUT_DATA_SIZE       16    // Number of bytes coming from the master
#define OUTPUT_DATA_SIZE      16    // Number of bytes going to master
#define VENDOR_DATA_SIZE      0     // Number of bytes for manufacturer-specific data
#define EXT_DIAG_DATA_SIZE    0     // Number of bytes for extended diagnostics

#define OUTPUT_MODULE_CNT     1     // Number of output modules
#define INPUT_MODULE_CNT      1     // Number of input modules
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
// Profibus process control
///////////////////////////////////////////////////////////////////////////////////////////////////
#define PROFIBUS_WAIT_SYN     1
#define PROFIBUS_WAIT_DATA    2
#define PROFIBUS_GET_DATA     3
#define PROFIBUS_SEND_DATA    4
///////////////////////////////////////////////////////////////////////////////////////////////////
unsigned char uart_buffer[MAX_BUFFER_SIZE];
unsigned int  uart_byte_cnt = 0;
unsigned int  uart_byte_cnt0 = 0;
unsigned int  uart_transmit_cnt = 0;
unsigned int  uart_transmit_cnt0 = 0;
unsigned long lastmillis;
// Profibus Flags and Variables
unsigned char profibus_status;
unsigned char diagnose_status;
unsigned char slave_addr;
unsigned char master_addr;
unsigned char group;
volatile unsigned char new_data;

#if (OUTPUT_DATA_SIZE > 0)
volatile unsigned char Profibus_out_register[OUTPUT_DATA_SIZE];
#endif
#if (INPUT_DATA_SIZE > 0)
unsigned char Profibus_in_register [INPUT_DATA_SIZE];
#endif
#if (VENDOR_DATA_SIZE > 0)
unsigned char Vendor_Data[VENDOR_DATA_SIZE];
#endif
#if (EXT_DIAG_DATA_SIZE > 0)
unsigned char Diag_Data[EXT_DIAG_DATA_SIZE];
#endif
unsigned char Input_Data_size;
unsigned char Output_Data_size;
unsigned char Vendor_Data_size;   // Number of read-in manufacturer-specific bytes

uint8_t rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void init_Profibus (void);
void profibus_RX (void);
void profibus_send_CMD (unsigned char type,unsigned char function_code,unsigned char sap_offset,char *pdu,unsigned char length_pdu);
void profibus_TX (char *data, unsigned char length);
unsigned char checksum(char *data, unsigned char length);
unsigned char addmatch (unsigned char destination);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_Profibus (void)
{
  unsigned char cnt;
  // Variable init
  profibus_status = PROFIBUS_WAIT_SYN;  // Wait at least Tsyn until allowing RXdata
  diagnose_status = false;
  Input_Data_size = 0;
  Output_Data_size = 0;
  Vendor_Data_size = 0;
  group = 0;

  slave_addr = SLAVE_ADDRESS;  // <<< Temporary address assignment.
  // TODO: Read address from EEPROM or switches.
  // Illegal addresses are forced to DEFAULT (126). Set Address can be used to change it.
  if((slave_addr == 0) || (slave_addr > 126))
    slave_addr = DEFAULT_ADD;

  // Clear data
  #if (OUTPUT_DATA_SIZE > 0)
  for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
  {
    Profibus_out_register[cnt] = 0xFF;
  }
  #endif
  #if (INPUT_DATA_SIZE > 0)
  for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
  {
    Profibus_in_register[cnt] = 0x00;
  }
  #endif
  #if (VENDOR_DATA_SIZE > 0)
  for (cnt = 0; cnt < VENDOR_DATA_SIZE; cnt++)
  {
    Vendor_Data[cnt] = 0x00;
  }
  #endif
  #if (DIAG_DATA_SIZE > 0)
  for (cnt = 0; cnt < DIAG_DATA_SIZE; cnt++)
  {
    Diag_Data[cnt] = 0x00;
  }
  #endif
  new_data=0;
  //__disable_irq();           // Disable all interrupts
  HAL_UART_Transmit_IT(&huart1, &uart_buffer, 1);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // Prepara para receber 1 byte
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIMEOUT_MAX_SYN_TIME);
  HAL_TIM_Base_Start_IT(&htim2);
 // __enable_irq();             // Enable all interrupts
}

void profibus_RX (void)
{
  unsigned char cnt;
  unsigned char telegramm_type;
  unsigned char process_data;

  // Profibus data types
  unsigned char destination_add;
  unsigned char source_add;
  unsigned char function_code;
  unsigned char FCS_data;   // Frame Check Sequence
  unsigned char PDU_size;   // PDU Size
  unsigned char DSAP_data;  // SAP Destination
  unsigned char SSAP_data;  // SAP Source

  process_data = false;
  telegramm_type = uart_buffer[0];

  switch (telegramm_type)
  {
    case SD1: // Telegram without data, max. 6 bytes

        if (uart_byte_cnt != 6) break;

        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        function_code   = uart_buffer[3];
        FCS_data        = uart_buffer[4];

        if (addmatch(destination_add)       == false) break;
        if (checksum(&uart_buffer[1], 3) != FCS_data) break;

        process_data = true;

        break;

    case SD2: // Telegram with variable data length

        if (uart_byte_cnt != uart_buffer[1] + 6) break;

        PDU_size        = uart_buffer[1];
        destination_add = uart_buffer[4];
        source_add      = uart_buffer[5];
        function_code   = uart_buffer[6];
        FCS_data        = uart_buffer[PDU_size + 4];

        if (addmatch(destination_add)              == false) break;
        if (checksum(&uart_buffer[4], PDU_size) != FCS_data) break;

        process_data = true;

        break;

    case SD3: // Telegram with 5 bytes data, max. 11 bytes
        if (uart_byte_cnt != 11) break;
        PDU_size        = 8;              // DA + SA + FC + PDU
        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        function_code   = uart_buffer[3];
        FCS_data        = uart_buffer[9];
        if (addmatch(destination_add)       == false) break;
        if (checksum(&uart_buffer[1], 8) != FCS_data) break;
        process_data = true;
        break;
    case SD4: // Token with 3 Byte Data
        if (uart_byte_cnt != 3) break;
        destination_add = uart_buffer[1];
        source_add      = uart_buffer[2];
        if (addmatch(destination_add)       == false) break;
        break;
    default:
        break;
  }

  // Only evaluate if data is OK
  if (process_data == true)
  {
    master_addr = source_add; // Master address is Source address
    //Service Access Point detected?
    if ((destination_add & 0x80) && (source_add & 0x80))
    {
      DSAP_data = uart_buffer[7];
      SSAP_data = uart_buffer[8];
      // 1) SSAP 62 -> DSAP 60 (Get Diagnostics Request)
      // 2) SSAP 62 -> DSAP 61 (Set Parameters Request)
      // 3) SSAP 62 -> DSAP 62 (Check Config Request)
      // 4) SSAP 62 -> DSAP 60 (Get Diagnostics Request)
      // 5) Data Exchange Request (normal cycle)
      switch (DSAP_data)
      {
        case SAP_SET_SLAVE_ADR: // Set Slave Address (SSAP 62 -> DSAP 55)
            // Only possible in the "Wait Parameter" (WPRM) state
            slave_addr = uart_buffer[9];
            profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            break;
        case SAP_GLOBAL_CONTROL: // Global Control Request (SSAP 62 -> DSAP 58)
            // If "Clear Data" high, then PLC CPU on "Stop"
            if (uart_buffer[9] & CLEAR_DATA_)
            {
              LED_ERROR_ON;  // Status "PLC not ready"
            }
            else
            {
              LED_ERROR_OFF; // Status "PLC OK"
            }

            // Calculate group
            for (cnt = 0;  uart_buffer[10] != 0; cnt++) uart_buffer[10]>>=1;

            // If command is for us
            if (cnt == group)
            {
              if (uart_buffer[9] & UNFREEZE_)
              {
                // Delete FREEZE state
              }
              else if (uart_buffer[9] & UNSYNC_)
              {
                // Delete SYNC state
              }
              else if (uart_buffer[9] & FREEZE_)
              {
                // Do not read inputs again
              }
              else if (uart_buffer[9] & SYNC_)
              {
                // Set outputs only with SYNC command
              }
            }

            break;

        case SAP_SLAVE_DIAGNOSIS: // Get Diagnostics Request (SSAP 62 -> DSAP 60)

              // After receiving the diagnosis, the DP slave changes state
              // "Power on Reset" (POR) in the state "Wait Parameter" (WPRM)
              // At the end of initialization ("Data Exchange" state (DXCHG))
              // the master sends a Diagnostics Request a second time to check correct configuration
            if (function_code == (REQUEST_ + FCB_ + SRD_HIGH))
            {
              uart_buffer[7]  = SSAP_data;                    // Target SAP Master
              uart_buffer[8]  = DSAP_data;                    // Source SAP Slave
              uart_buffer[9]  = STATION_NOT_READY_;           // Status 1
              uart_buffer[10] = STATUS_2_DEFAULT + PRM_REQ_;  // Status 2
              uart_buffer[11] = DIAG_SIZE_OK;                 // Status 3
              uart_buffer[12] = MASTER_ADD_DEFAULT;           // Address Master
              uart_buffer[13] = IDENT_HIGH_BYTE;              // Ident high
              uart_buffer[14] = IDENT_LOW_BYTE;               // Ident low
              #if (EXT_DIAG_DATA_SIZE > 0)
              uart_buffer[15] = EXT_DIAG_DATA_SIZE;           // Device-related diagnosis (number of bytes)
              for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
              {
                uart_buffer[16+cnt] = Diag_Data[cnt];
              }
              #endif
              profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET, &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
            }
            else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
                     function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
            {
              // Diagnostic request to check data from Check Config Request
              uart_buffer[7]  = SSAP_data;                    // Target SAP Master
              uart_buffer[8]  = DSAP_data;                    // Source SAP slave
              if (diagnose_status == true)
                uart_buffer[9]  = EXT_DIAG_;                  // Status 1
              else
                uart_buffer[9]  = 0x00;
              uart_buffer[10] = STATUS_2_DEFAULT;             // Status 2
              uart_buffer[11] = DIAG_SIZE_OK;                 // Status 3
              uart_buffer[12] = master_addr - SAP_OFFSET;     // Address Master
              uart_buffer[13] = IDENT_HIGH_BYTE;              // Ident high
              uart_buffer[14] = IDENT_LOW_BYTE;               // Ident low
              #if (EXT_DIAG_DATA_SIZE > 0)
              uart_buffer[15] = EXT_DIAG_DATA_SIZE;           // Device-related diagnosis (number of bytes)
              for (cnt = 0; cnt < EXT_DIAG_DATA_SIZE; cnt++)
              {
                uart_buffer[16+cnt] = Diag_Data[cnt];
              }
              #endif

              profibus_send_CMD(SD2, DATA_LOW, SAP_OFFSET, &uart_buffer[7], 8 + EXT_DIAG_DATA_SIZE);
            }

            break;

        case SAP_SET_PRM: // Set Parameters Request (SSAP 62 -> DSAP 61)

            // After receiving the parameters, the DP slave changes state
            // "Wait Parameter" (WPRM) in the state "Wait Configuration" (WCFG)

            // Only accept data for our device
            if ((uart_buffer[13] == IDENT_HIGH_BYTE) && (uart_buffer[14] == IDENT_LOW_BYTE))
            {
              // User Parameter Size = Length - DA, SA, FC, DSAP, SSAP, 7 Parameter Bytes
              Vendor_Data_size = PDU_size - 12;
              // Read in user parameters
              #if (VENDOR_DATA_SIZE > 0)
              for (cnt = 0; cnt < Vendor_Data_size; cnt++) Vendor_Data[cnt] = uart_buffer[16+cnt];
              #endif
              // Read group
              for (group = 0; uart_buffer[15] != 0; group++) uart_buffer[15]>>=1;
              profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            }
            break;

        case SAP_CHK_CFG: // Check Config Request (SSAP 62 -> DSAP 62)

        // After receiving the configuration, the DP slave changes state
        // "Wait Configuration" (WCFG) in the "Data Exchange" state (DXCHG)
        // IO configuration:
        // Compact format for max. 16/32 bytes IO
        // special format for max. 64/132 bytes IO
        // evaluate several bytes depending on the PDU data size
        // LE / LEr - (DA + SA + FC + DSAP + SSAP) = number of config bytes
      Output_Data_size=0;
      Input_Data_size=0;
            for (cnt = 0; cnt < uart_buffer[1] - 5; cnt++)
            {
              switch (uart_buffer[9+cnt] & CFG_DIRECTION_)
              {
                case CFG_INPUT:
                    Input_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                      Input_Data_size += Input_Data_size*2;
                    break;

                case CFG_OUTPUT:
                    Output_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                      Output_Data_size += Output_Data_size*2;
                    break;

                case CFG_INPUT_OUTPUT:
                    Input_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    Output_Data_size += (uart_buffer[9+cnt] & CFG_BYTE_CNT_) + 1;
                    if (uart_buffer[9+cnt] & CFG_WIDTH_ & CFG_WORD)
                    {
                      Input_Data_size += Input_Data_size*2;
                      Output_Data_size += Output_Data_size*2;
                    }
                    break;

                case CFG_SPECIAL:
                    // Special format
                    // Manufacturer-specific bytes available?
                    if (uart_buffer[9+cnt] & CFG_SP_VENDOR_CNT_)
                    {
                      // Save the number of manufacturer data
                      Vendor_Data_size = uart_buffer[9+cnt] & CFG_SP_VENDOR_CNT_;
                      // Deduct number of total
                      uart_buffer[1] -= Vendor_Data_size;
                    }

                    // I/O Data
                    switch (uart_buffer[9+cnt] & CFG_SP_DIRECTION_)
                    {
                      case CFG_SP_VOID:
                          // Empty data field
                          break;

                      case CFG_SP_INPUT:
                          Input_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Input_Data_size += Input_Data_size*2;
                          cnt++;  // We already have this byte
                          break;

                      case CFG_SP_OUTPUT:
                          Output_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Output_Data_size += Output_Data_size*2;
                          cnt++;  //We already have this byte
                          break;

                      case CFG_SP_INPUT_OPTPUT:
                          // Erst Ausgang...
                          Output_Data_size += (uart_buffer[10+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[10+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Output_Data_size += Output_Data_size*2;
                          // Dann Eingang
                          Input_Data_size += (uart_buffer[11+cnt] & CFG_SP_BYTE_CNT_) + 1;
                          if (uart_buffer[11+cnt] & CFG_WIDTH_ & CFG_WORD)
                            Input_Data_size += Input_Data_size*2;
                          cnt += 2;  // We already have these bytes
                          break;

                    } // Switch End
                    break;

                default:
                    Input_Data_size = 0;
                    Output_Data_size = 0;
                    break;

              } // Switch End
            } // For End

            if (Vendor_Data_size != 0)
            {
              // Evaluate
            }
            //In case of error -> send CFG_FAULT_ to diagnosis
            // short acknowledgment
            profibus_send_CMD(SC, 0, SAP_OFFSET, &uart_buffer[0], 0);
            break;

        default:
            // Unknown SAP
            break;
      } //Switch DSAP_data end
    }
    // Destination: Slave address
    else if (destination_add == slave_addr)
    {
      // Status query
      if (function_code == (REQUEST_ + FDL_STATUS))
      {
        profibus_send_CMD(SD1, FDL_STATUS_OK, 0, &uart_buffer[0], 0);
      }
      // Master sends output data and requests input data(Send and Request Data)
      else if (function_code == (REQUEST_ + FCV_ + SRD_HIGH) ||
               function_code == (REQUEST_ + FCV_ + FCB_ + SRD_HIGH))
      {

        // Read data from master
        #if (INPUT_DATA_SIZE > 0)
        for (cnt = 0; cnt < INPUT_DATA_SIZE; cnt++)
        {
          Profibus_in_register[cnt] = uart_buffer[cnt + 7];
      new_data=1;
        }
        #endif


        // Write data for master in buffer
        #if (OUTPUT_DATA_SIZE > 0)
        for (cnt = 0; cnt < OUTPUT_DATA_SIZE; cnt++)
        {
          uart_buffer[cnt + 7] = Profibus_out_register[cnt];
        }
        #endif


        #if (OUTPUT_DATA_SIZE > 0)
        if (diagnose_status == true)
          profibus_send_CMD(SD2, DIAGNOSE, 0, &uart_buffer[7], 0);  // Request a diagnosis
        else
          profibus_send_CMD(SD2, DATA_LOW, 0, &uart_buffer[7], Input_Data_size);  // send data
        #else
        if (diagnose_status == true)
          profibus_send_CMD(SD1, DIAGNOSE, 0, &uart_buffer[7], 0);  // Request a diagnosis
        else
          profibus_send_CMD(SC, 0, 0, &uart_buffer[7], 0);          // short acknowledgment
        #endif
      }
    }

  } //Data valid at the end

}

void profibus_send_CMD (unsigned char type,
                        unsigned char function_code,
                        unsigned char sap_offset,
                        char *pdu,
                        unsigned char length_pdu)
{
  unsigned char length_data;


  switch(type)
  {
    case SD1:
      uart_buffer[0] = SD1;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      uart_buffer[3] = function_code;
      uart_buffer[4] = checksum(&uart_buffer[1], 3);
      uart_buffer[5] = ED;
      length_data = 6;
      break;

    case SD2:
      uart_buffer[0] = SD2;
      uart_buffer[1] = length_pdu + 3;  // Length of the PDU incl. DA, SA and FC
      uart_buffer[2] = length_pdu + 3;
      uart_buffer[3] = SD2;
      uart_buffer[4] = master_addr;
      uart_buffer[5] = slave_addr + sap_offset;
      uart_buffer[6] = function_code;
      //Data is already filled in before the function is called
      uart_buffer[7+length_pdu] = checksum(&uart_buffer[4], length_pdu + 3);
      uart_buffer[8+length_pdu] = ED;
      length_data = length_pdu + 9;
      break;

    case SD3:
      uart_buffer[0] = SD3;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      uart_buffer[3] = function_code;
      // Data is already filled in before the function is called
      uart_buffer[9] = checksum(&uart_buffer[4], 8);
      uart_buffer[10] = ED;
      length_data = 11;
      break;

    case SD4:
      uart_buffer[0] = SD4;
      uart_buffer[1] = master_addr;
      uart_buffer[2] = slave_addr + sap_offset;
      length_data = 3;
      break;

    case SC:
      uart_buffer[0] = SC;
      length_data = 1;
      break;

    default:
      break;
  }
  profibus_TX(&uart_buffer[0], length_data);

}


void profibus_TX (char *data, unsigned char length)
{
  TX_ENABLE_ON;  // Enable Transmit (Switch to Transmit)
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TIMEOUT_MAX_TX_TIME);
  profibus_status = PROFIBUS_SEND_DATA;

  uart_byte_cnt = length;         // Number of bytes to send
  uart_transmit_cnt = 0;          // Payer for sent bytes
  __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reinicia a contagem (equivalente a TCNT1 = 0)
  HAL_UART_Transmit_IT(&huart1, (uint8_t*)data, length);
}
///////////////////////////////////////////////////////////////////////////////////////////////////


unsigned char checksum(char *data, unsigned char length)
{
  unsigned char csum = 0;

  while(length--)
  {
    csum += data[length];
  }

  return csum;
}


unsigned char addmatch (unsigned char destination)
{
  if ((destination != slave_addr) &&                // Slave
      (destination != slave_addr + SAP_OFFSET) &&   // SAP Slave
      (destination != BROADCAST_ADD) &&             // Broadcast
      (destination != BROADCAST_ADD + SAP_OFFSET)){  // SAP Broadcast
        return false;
      }
  return true;
}

void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    // Verifica se foi o timer que queremos (TIM1)
    if (htim->Instance == TIM2)
    {

        // O HAL já limpou a flag de interrupção (não precisa "TIMER1_STOP")

    	//->

        switch (profibus_status)
        {

          case PROFIBUS_WAIT_SYN: // TSYN expired
            profibus_status = PROFIBUS_WAIT_DATA;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TIMEOUT_MAX_SDR_TIME);
            uart_byte_cnt = 0;
            break;


          case PROFIBUS_SEND_DATA: // Transmission timeout
            profibus_status = PROFIBUS_WAIT_SYN;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TIMEOUT_MAX_SYN_TIME);
            TX_ENABLE_OFF; // Usa a macro do HAL
            break;

          default:
            break;
        }

        // O "TIMER1_RUN" e "TCNT1 = 0" no final da ISR do AVR
        // é para rearmar o timer. No STM32, o timer continua rodando.
        // Você só precisa zerar o contador se não estiver em modo "auto-reload".
        // __HAL_TIM_SET_COUNTER(htim, 0); // Equivalente a TCNT1 = 0
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)  // ou USART2, conforme usado
  {
	  HAL_GPIO_TogglePin(GPIOC, LED_PIN);
    uart_buffer[uart_byte_cnt] = rx_byte;

    if (profibus_status == PROFIBUS_WAIT_DATA)
      profibus_status = PROFIBUS_GET_DATA;

    if (profibus_status == PROFIBUS_GET_DATA)
    {
      uart_byte_cnt++;
      if (uart_byte_cnt >= MAX_BUFFER_SIZE) uart_byte_cnt--;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0); // zera o timer, equivalente ao TCNT1 = 0;

    // Reinicia recepção para o próximo byte
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
	  HAL_GPIO_TogglePin(GPIOC, LED_PIN);
    if (uart_transmit_cnt < uart_byte_cnt)
    {
      HAL_UART_Transmit_IT(&huart1, &uart_buffer[uart_transmit_cnt++], 1);
    }
    else
    {
      // Tudo enviado
      uart_transmit_cnt = 0;
      uart_byte_cnt = 0;
    }
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Zera o timer
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  init_Profibus();

   TX_ENABLE_OFF; // Disable Transmit (Switch to Recieve)
  samplingtime = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  //if ( (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - samplingtime) > 10 )
	  if ( (unsigned long) (HAL_GetTick() * 1000 - samplingtime) > 10 )
	 	  	 	    {
	  // 1. Acende o LED (define o pino como nível BAIXO)
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	      // 2. Espera 200 milissegundos
	      HAL_Delay(200);

	      // 3. Apaga o LED (define o pino como nível ALTO)
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

	      // 4. Espera 200 milissegundos
	      HAL_Delay(200);
	 	  	 	    }
	 	  	 	    */

	  //if ( (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - samplingtime) > 10 )
	  if ( (unsigned long) (HAL_GetTick() * 1000 - samplingtime) > 10 )
	  	 	    {
	  	 	      if (HAL_GPIO_ReadPin(GPIOA, TOUCH_PIN) == GPIO_PIN_RESET)
	  	 	      {
	  	 	    	//HAL_GPIO_WritePin(GPIOC, LED_PIN,GPIO_PIN_RESET);
	  	 		  //HAL_GPIO_WritePin(GPIOC, LED_PIN,GPIO_PIN_RESET);
	  	 	      Profibus_out_register[0]=0X01;
	  	 	      }
	  	 	      else
	  	 	      {
	  	 	    	//HAL_GPIO_WritePin(GPIOC, LED_PIN,GPIO_PIN_SET);
	  	 		  //HAL_GPIO_WritePin(GPIOC, LED_PIN,GPIO_PIN_SET);
	  	 	      Profibus_out_register[0]=0X00;

	  	 	      }
	  	 	      samplingtime = __HAL_TIM_GET_COUNTER(&htim2);
	  	 	    }

	  	 	    if(new_data==1)
	  	 	    {
	  	 	        Profibus_out_register[0] +=new_data;
	  	 	        new_data=0;
	  	 	        Profibus_out_register[0] = Profibus_in_register[0];
	  	 	      }
	  	 	    //HAL_GPIO_WritePin(GPIOC, LED_PIN, ((Profibus_in_register[0] & 0x01) == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 45450;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
