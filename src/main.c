#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "lcd.h"
#include <stdio.h>
#include "BMP280.h"
#include <math.h>

// -------------------- SENSOR BMP280 --------------------
BMP280_HandleTypedef bmp280;
float pressure, temperature, humidity;
char buffer[16];

// -------------------- SENSOR DHT11 --------------------
#define DHT11_PORT GPIOB
#define DHT11_PIN GPIO_PIN_9
uint8_t RHI, RHD, TCI, TCD, SUM;
uint32_t pMillis, cMillis;
float tCelsius = 0;
float RH = 0;

// -------------------- DELAYS --------------------
void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

void miliDelay(uint16_t delay_ms) {
    while (delay_ms--) {
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        while (__HAL_TIM_GET_COUNTER(&htim1) < 1000);
    }
}

// -------------------- BOTONES --------------------
typedef enum {
    BOTON_LIBRE,
    BOTON_PRESIONADO,
    BOTON_ESPERANDO_LIBERACION
} EstadoBoton;

typedef struct {
    EstadoBoton estado;
    uint32_t tiempo_presionado;
    bool evento;
} Boton;

Boton boton0 = {BOTON_LIBRE, 0, false};
Boton boton1 = {BOTON_LIBRE, 0, false};
Boton boton2 = {BOTON_LIBRE, 0, false};

void actualizar_boton(Boton* b, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
    switch (b->estado) {
        case BOTON_LIBRE:
            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                b->tiempo_presionado = HAL_GetTick();
                b->estado = BOTON_PRESIONADO;
            }
            break;
        case BOTON_PRESIONADO:
            if (HAL_GetTick() - b->tiempo_presionado >= 10) {
                if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_SET) {
                    b->estado = BOTON_ESPERANDO_LIBERACION;
                } else {
                    b->estado = BOTON_LIBRE;
                }
            }
            break;
        case BOTON_ESPERANDO_LIBERACION:
            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
                b->evento = true;
                b->estado = BOTON_LIBRE;
            }
            break;
    }
}

// -------------------- DHT11 --------------------
uint8_t DHT11_Start(void) {
    uint8_t Response = 0;
    GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
    GPIO_InitStructPrivate.Pin = DHT11_PIN;
    GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);
    miliDelay(20);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);
    microDelay(30);
    GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate);
    microDelay(40);
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
        microDelay(80);
        if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) Response = 1;
    }
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
        cMillis = HAL_GetTick();
    }
    return Response;
}

uint8_t DHT11_Read(void) {
    uint8_t a, b = 0;
    for (a = 0; a < 8; a++) {
        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
            cMillis = HAL_GetTick();
        }
        microDelay(40);
        if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
            b &= ~(1 << (7 - a));
        else
            b |= (1 << (7 - a));
        pMillis = HAL_GetTick();
        cMillis = HAL_GetTick();
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
            cMillis = HAL_GetTick();
        }
    }
    return b;
}

// -------------------- VISUALIZACIÓN --------------------
uint8_t modo_visualizacion = 0;
unsigned long tiempo = 0, tiempo2 = 0;

void SystemClock_Config(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C2_Init();
    MX_TIM1_Init();
    Lcd_Init();
    HAL_TIM_Base_Start(&htim1);

    bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = &hi2c2;

    while (!bmp280_init(&bmp280, &bmp280.params)) {
        Lcd_Clear();
        Lcd_Set_Cursor(1, 1);
        Lcd_Send_String("Error Init");
        Lcd_Set_Cursor(2, 1);
        Lcd_Send_String("Reintentando...");
        miliDelay(2000);
    }

    Lcd_Clear();
    Lcd_Set_Cursor(1, 1);
    Lcd_Send_String("Estacion");
    Lcd_Set_Cursor(2, 1);
    Lcd_Send_String("Meteorologica");
    miliDelay(2000);
    Lcd_Clear();

    while (1) {
    char templcd[16] = {0};
    char humlcd[16] = {0};        
        tiempo = HAL_GetTick();

        // Actualizar botones
        actualizar_boton(&boton0, GPIOA, GPIO_PIN_0);
        actualizar_boton(&boton1, GPIOA, GPIO_PIN_1);
        actualizar_boton(&boton2, GPIOA, GPIO_PIN_2);

        if (boton0.evento) {
            modo_visualizacion = 0;
            boton0.evento = false;
        }
        if (boton1.evento) {
            modo_visualizacion = 1;
            boton1.evento = false;
        }
        if (boton2.evento) {
            modo_visualizacion = 2;
            boton2.evento = false;
        }

        // Leer sensores cada 2000 ms
        if (tiempo - tiempo2 >= 2000) {
            tiempo2 = tiempo;
            Lcd_Clear();
                    if (DHT11_Start()) {
                        RHI = DHT11_Read();
                        RHD = DHT11_Read();
                        TCI = DHT11_Read();
                        TCD = DHT11_Read();
                        SUM = DHT11_Read();
                        if ((RHI + RHD + TCI + TCD) == SUM) {
                            tCelsius = (float)TCI + (float)(TCD / 10);
                            RH = (float)RHI + (float)(RHD / 10);

                        } else {
                            Lcd_Set_Cursor(1, 1);
                            Lcd_Send_String("Error DHT11");
                            Lcd_Set_Cursor(2, 1);
                            Lcd_Send_String("Checksum fail");
                        }
                    } else {
                        Lcd_Set_Cursor(1, 1);
                        Lcd_Send_String("No respuesta");
                        Lcd_Set_Cursor(2, 1);
                        Lcd_Send_String("DHT11");
                    }
                }
 if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
                        Lcd_Set_Cursor(1, 1);
                        Lcd_Send_String("Error lectura");
                        Lcd_Set_Cursor(2, 1);
                        Lcd_Send_String("BMP280");
                    } else {
                        Lcd_Set_Cursor(1, 1);
                        sprintf(buffer, "Pres: %.2f hPa", pressure / 100);
                        Lcd_Send_String(buffer);
                        Lcd_Set_Cursor(2, 1);
                        sprintf(buffer, "Temp: %.2f C", temperature);
                        Lcd_Send_String(buffer);
                    }                
            switch (modo_visualizacion) {
                case 0:

                            Lcd_Set_Cursor(1, 1);
                            sprintf(templcd, "Temp: %.2f C", tCelsius);
                            Lcd_Send_String(templcd);
                            Lcd_Set_Cursor(2, 1);
                            sprintf(humlcd, "Humedad: %.2f%%", RH);
                            Lcd_Send_String(humlcd);
                    break;

                case 1:
                   
                break;
                case 2:
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Send_String("Modo Personal");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Send_String("Boton 2 activo");
                    break;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
