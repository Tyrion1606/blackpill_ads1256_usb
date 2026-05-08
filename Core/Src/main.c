/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Leitura ADS1256 via SPI2 e envio via USB CDC
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
#define APP_START_ADDRESS 0x08004000U

/* Pinos do ADS1256 */
#define ADS_CS_PORT        GPIOB
#define ADS_CS_PIN         GPIO_PIN_12

#define ADS_DRDY_PORT      GPIOB
#define ADS_DRDY_PIN       GPIO_PIN_1

#define ADS_PDWN_PORT      GPIOB
#define ADS_PDWN_PIN       GPIO_PIN_0

/* LED onboard da Blackpill */
#define LED_PORT           GPIOC
#define LED_PIN            GPIO_PIN_13

/* Comandos ADS1256 */
#define ADS_CMD_WAKEUP     0x00
#define ADS_CMD_RDATA      0x01
#define ADS_CMD_RDATAC     0x03
#define ADS_CMD_SDATAC     0x0F
#define ADS_CMD_RREG       0x10
#define ADS_CMD_WREG       0x50
#define ADS_CMD_SELFCAL    0xF0
#define ADS_CMD_SYNC       0xFC
#define ADS_CMD_STANDBY    0xFD
#define ADS_CMD_RESET      0xFE

/* Registradores ADS1256 */
#define ADS_REG_STATUS     0x00
#define ADS_REG_MUX        0x01
#define ADS_REG_ADCON      0x02
#define ADS_REG_DRATE      0x03

/* Taxas de amostragem ADS1256 */
#define ADS_DRATE_30000SPS 0xF0
#define ADS_DRATE_15000SPS 0xE0
#define ADS_DRATE_7500SPS  0xD0
#define ADS_DRATE_1000SPS  0xA1
#define ADS_DRATE_100SPS   0x82

/*
 * Canal inicial:
 * AIN0 contra AINCOM.
 */
#define ADS_MUX_AIN0_AINCOM 0x08

/*
 * Comece com 1000 SPS para validar.
 * Depois troque para ADS_DRATE_30000SPS.
 *
 * Atenção: imprimir texto via USB CDC pode não acompanhar 30000 SPS.
 */
#define ADS_DRATE_SELECTED  ADS_DRATE_15000SPS

#define USB_TX_BUF_SIZE 8192

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void DWT_Init(void);
static void delay_us(uint32_t us);

static void usb_write_blocking(const char *data, uint16_t len);

static void ads_cs_low(void);
static void ads_cs_high(void);

static void ads_send_cmd(uint8_t cmd);
static void ads_write_reg(uint8_t reg, uint8_t value);
static uint8_t ads_read_reg(uint8_t reg);

static uint8_t ads_wait_drdy_timeout(uint32_t timeout_ms);
static void ads_wait_drdy(void);

static void ads_init(void);
static int32_t ads_read_raw24_continuous(void);

static void error_blink_fast(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /*
   * Como seu app está em 0x08004000 por causa do bootloader WeAct,
   * a tabela de vetores precisa apontar para 0x08004000.
   */
  SCB->VTOR = APP_START_ADDRESS;
  __DSB();
  __ISB();

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();

  DWT_Init();

  /*
   * Estado inicial seguro dos pinos do ADS1256.
   */
  HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET);      // CS inativo
  HAL_GPIO_WritePin(ADS_PDWN_PORT, ADS_PDWN_PIN, GPIO_PIN_SET);  // ADS acordado
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);            // LED aceso

  /*
   * Aguarda o PC reconhecer a porta USB CDC.
   * No Linux deve aparecer algo como /dev/ttyACM0.
   */
  HAL_Delay(2000);

  usb_write_blocking("Blackpill STM32F411 + ADS1256\r\n",
                     strlen("Blackpill STM32F411 + ADS1256\r\n"));

  usb_write_blocking("Formato CSV: seq,raw\r\n",
                     strlen("Formato CSV: seq,raw\r\n"));

  usb_write_blocking("Inicializando ADS1256...\r\n",
                     strlen("Inicializando ADS1256...\r\n"));

  ads_init();

  usb_write_blocking("ADS1256 OK. Iniciando leitura.\r\n",
                     strlen("ADS1256 OK. Iniciando leitura.\r\n"));

  uint32_t seq = 0;
  char txbuf[USB_TX_BUF_SIZE];
  uint16_t txpos = 0;

  while (1)
  {
    /*
     * Aguarda nova amostra.
     * DRDY é ativo em LOW.
     */
    ads_wait_drdy();

    /*
     * Lê amostra bruta signed de 24 bits.
     */
    int32_t raw = ads_read_raw24_continuous();

    /*
     * Formata como CSV:
     * seq,raw
     */
    int n = snprintf(&txbuf[txpos],
                     USB_TX_BUF_SIZE - txpos,
                     "%lu,%ld\r\n",
                     (unsigned long)seq,
                     (long)raw);

    if (n > 0)
    {
      txpos += (uint16_t)n;
    }

    seq++;

    /*
     * Envia em blocos, em vez de enviar linha por linha.
     * Isso reduz overhead do USB CDC.
     */
    if (txpos > (USB_TX_BUF_SIZE - 64))
    {
      usb_write_blocking(txbuf, txpos);
      txpos = 0;

      /*
       * LED C13 da Blackpill geralmente é ativo em LOW.
       * Toggle indica que dados estão sendo enviados.
       */
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*
   * Blackpill com cristal externo de 25 MHz.
   *
   * HSE  = 25 MHz
   * PLLM = 25  -> 1 MHz
   * PLLN = 192 -> 192 MHz
   * PLLP = 2   -> SYSCLK 96 MHz
   * PLLQ = 4   -> USB 48 MHz
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // 96 MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // 48 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // 96 MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < ticks)
  {
    __NOP();
  }
}

static void usb_write_blocking(const char *data, uint16_t len)
{
  uint32_t start = HAL_GetTick();

  while (CDC_Transmit_FS((uint8_t *)data, len) == USBD_BUSY)
  {
    if ((HAL_GetTick() - start) > 1000)
    {
      return;
    }
  }
}

static void ads_cs_low(void)
{
  HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_RESET);
}

static void ads_cs_high(void)
{
  HAL_GPIO_WritePin(ADS_CS_PORT, ADS_CS_PIN, GPIO_PIN_SET);
}

static void ads_send_cmd(uint8_t cmd)
{
  ads_cs_low();

  if (HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    ads_cs_high();
    Error_Handler();
  }

  ads_cs_high();

  delay_us(10);
}

static void ads_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx[3];

  tx[0] = ADS_CMD_WREG | (reg & 0x0F);
  tx[1] = 0x00;
  tx[2] = value;

  ads_cs_low();

  if (HAL_SPI_Transmit(&hspi2, tx, 3, HAL_MAX_DELAY) != HAL_OK)
  {
    ads_cs_high();
    Error_Handler();
  }

  ads_cs_high();

  delay_us(10);
}

static uint8_t ads_read_reg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t value = 0;

  tx[0] = ADS_CMD_RREG | (reg & 0x0F);
  tx[1] = 0x00;

  ads_cs_low();

  if (HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY) != HAL_OK)
  {
    ads_cs_high();
    Error_Handler();
  }

  delay_us(10);

  if (HAL_SPI_Receive(&hspi2, &value, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    ads_cs_high();
    Error_Handler();
  }

  ads_cs_high();

  delay_us(10);

  return value;
}

static uint8_t ads_wait_drdy_timeout(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return 0;
    }
  }

  return 1;
}

static void ads_wait_drdy(void)
{
  while (HAL_GPIO_ReadPin(ADS_DRDY_PORT, ADS_DRDY_PIN) == GPIO_PIN_SET)
  {
    __NOP();
  }
}

static void ads_init(void)
{
  ads_cs_high();

  /*
   * Mantém PDWN alto: ADS1256 ativo.
   */
  HAL_GPIO_WritePin(ADS_PDWN_PORT, ADS_PDWN_PIN, GPIO_PIN_SET);
  HAL_Delay(50);

  /*
   * Reset por comando SPI, pois seu módulo não expõe pino RESET.
   */
  ads_send_cmd(ADS_CMD_RESET);
  HAL_Delay(5);

  if (!ads_wait_drdy_timeout(1000))
  {
    usb_write_blocking("ERRO: timeout DRDY apos RESET\r\n",
                       strlen("ERRO: timeout DRDY apos RESET\r\n"));
    Error_Handler();
  }

  /*
   * Garante que não estamos em modo leitura contínua antes de configurar.
   */
  ads_send_cmd(ADS_CMD_SDATAC);
  HAL_Delay(2);

  /*
   * Seleciona canal AIN0 contra AINCOM.
   */
  ads_write_reg(ADS_REG_MUX, ADS_MUX_AIN0_AINCOM);

  /*
   * ADCON:
   * 0x00 = clock out off, sensor detect off, PGA gain = 1.
   */
  ads_write_reg(ADS_REG_ADCON, 0x00);

  /*
   * Configura taxa de amostragem.
   */
  ads_write_reg(ADS_REG_DRATE, ADS_DRATE_SELECTED);

  /*
   * Calibração interna.
   */
  ads_send_cmd(ADS_CMD_SELFCAL);

  if (!ads_wait_drdy_timeout(1000))
  {
    usb_write_blocking("ERRO: timeout DRDY apos SELFCAL\r\n",
                       strlen("ERRO: timeout DRDY apos SELFCAL\r\n"));
    Error_Handler();
  }

  /*
   * Verifica comunicação lendo de volta o registrador MUX.
   */
  uint8_t mux = ads_read_reg(ADS_REG_MUX);

  char msg[80];
  int n = snprintf(msg, sizeof(msg), "MUX lido: 0x%02X\r\n", mux);
  usb_write_blocking(msg, (uint16_t)n);

  if (mux != ADS_MUX_AIN0_AINCOM)
  {
    usb_write_blocking("ERRO: MUX diferente do esperado\r\n",
                       strlen("ERRO: MUX diferente do esperado\r\n"));
    Error_Handler();
  }

  /*
   * Entra em modo de leitura contínua.
   * Depois disso, a cada DRDY baixo, basta clockar 3 bytes pelo SPI.
   */
  ads_send_cmd(ADS_CMD_RDATAC);
  delay_us(10);
}

static int32_t ads_read_raw24_continuous(void)
{
  uint8_t tx[3] = {0xFF, 0xFF, 0xFF};
  uint8_t rx[3] = {0x00, 0x00, 0x00};

  ads_cs_low();

  if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3, HAL_MAX_DELAY) != HAL_OK)
  {
    ads_cs_high();
    Error_Handler();
  }

  ads_cs_high();

  int32_t value = ((int32_t)rx[0] << 16) |
                  ((int32_t)rx[1] << 8) |
                  ((int32_t)rx[2]);

  /*
   * Extensão de sinal de 24 bits para 32 bits.
   */
  if (value & 0x800000)
  {
    value |= 0xFF000000;
  }

  return value;
}

static void error_blink_fast(void)
{
  while (1)
  {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    HAL_Delay(80);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_Delay(80);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  error_blink_fast();
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
