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

// %% Auteur du programme : PESCATORE MENARD
 	 	 	 	 	 	 	 	 	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	       	         	    // 							CODE FINAL ATMOCPTV, partie 1 :				   //
									//contrôle des 3 capteurs météo, accéléromètre et transmissions UART Kikiwi//
	      	       	         	    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
// %% A l'ATTENTION DES UTILISATEURS : ce programme est fonctionnel et a été testé le 21/03/2025 en vue d'un lancement prévu initialement le 22/03/2025 %%
// %% A l'ATTENTION DES UTILISATEURS : la dernière modification des lignes de code date du 21/03/2025%%
// %% A l'ATTENTION DES UTILISATEURS : des lignes sont DELIBEREMENT présentes en double dans ce programme, NE PAS LES ENLEVER NI LES MODIFIER%%
// %% A l'ATTENTION DES UTILISATEURS : des lignes en doubles et les valeurs DELAIS ont été placés empiriquement pour optimiser la transmission Kikiwi %%
// %% A l'ATTENTION DES UTILISATEURS : toute modification des délais peut perturber gravement le processus de transmission par radio de la Kikiwi (la Kikiwi ne transmettrea plus les données des 3capteurs+ accéléromètre%%
// %% Pour toute question, vous pouvez me contacter via raphael.pescatore@gmail.com %%

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME.h"
#include "barometer.h"
#include <stdio.h>
#include <string.h>
#include "icm20948.h"
#include <string.h>
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
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
#define XON 0x11
/* USER CODE BEGIN PV */

axises my_gyro;
axises my_accel;
axises my_mag;

extern uint32_t finalpressure_ref;
extern uint32_t final_humidity_ref;
extern int finaltemp_ref;
extern int temperature_raw_ref;
extern int pressure_raw_ref;

extern uint32_t finalpressure_N;
extern uint32_t final_humidity_N;
extern int finaltemp_N;
extern int pressure_raw_N;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *data, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	    HAL_Init();
	    SystemClock_Config();

	    #define MAX_SAME_VALUE_COUNT 5

	    float last_accel_x = 0.0f;  // Dernière valeur de l'accéléromètre pour X
	    float last_accel_y = 0.0f;  // Dernière valeur de l'accéléromètre pour Y
	    int same_value_count_x = 0; // Compteur des mêmes valeurs pour X
	    int same_value_count_y = 0; // Compteur des mêmes valeurs pour Y
	    // Définition trames des données à envoyer à la carte Kikiwi
	    char buf2[48]; //75 octets, choix par sécurité, la taille est plus grande que 48 (limite Kikiwi)
	    uint8_t received_char; //1 octet, pour écouter les caractères XON avant d'envoyer nos mesures via le port série de la Kikiwi
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t data;
 		    printf("Coucou accéléromètre");
 		    icm20948_init();
 		     ak09916_init();
  /* USER CODE END 2 */

 		    /* Boucle infinie */
 		    /* USER CODE BEGIN WHILE */
 		   /* Configuration du capteur et initialisation des directions */
 		    HAL_Delay(1000);

 		    /* Activation, initialisation et désactivation de chaque capteur */

 		    BME280_CONFIG_SETUP_N();
 		      BME280_GET_COMP_VALS_N();
 		      HAL_Delay(1000);
 		    BME280_CONFIG_SETUP_ref();
 		    BME280_GET_COMP_VALS_ref();

 		    HAL_Delay(1000);




 		    Barometer_init_E();

 		    HAL_Delay(1000);

 		    BME280_CONFIG_SETUP_N();
 		        BME280_GET_COMP_VALS_N();


 		    printf("coucou Baromètre \n\r");
 		   int capture_count = 0;
	    while (1)
	    {
	        HAL_Delay(300);
	       // icm20948_wakeup();
	        // Lire les données des capteurs
	        icm20948_gyro_read_dps(&my_gyro);
	        icm20948_accel_read_g(&my_accel);
	        ak09916_mag_read_uT(&my_mag);

	        // Vérification pour l'axe X
	            if (my_accel.x == last_accel_x)
	            {
	                same_value_count_x++;
	            }
	            else
	            {
	                same_value_count_x = 0; // Réinitialiser le compteur si la valeur change
	            }

	            // Vérification pour l'axe Y
	            if (my_accel.y == last_accel_y)
	            {
	                same_value_count_y++;
	            }
	            else
	            {
	                same_value_count_y = 0; // Réinitialiser le compteur si la valeur change
	            }

	            // Si les valeurs de X et Y restent constantes pendant 5 itérations
	            if (same_value_count_x >= MAX_SAME_VALUE_COUNT && same_value_count_y >= MAX_SAME_VALUE_COUNT)
	            {
	                printf("Valeurs constantes détectées sur X et Y. Réinitialisation du programme.\n");

	                // Réinitialiser le capteur
	                icm20948_init();
	                ak09916_init();

	                // Réinitialiser les compteurs
	                same_value_count_x = 0;
	                same_value_count_y = 0;

	                // Réinitialisation éventuelle d'autres ressources si nécessaire
	            }

	            // Mettre à jour les dernières valeurs lues
	            last_accel_x = my_accel.x;
	            last_accel_y = my_accel.y;

	        // Afficher les données (ajouter vos fonctions d'affichage ici)
	        printf("Gyro: x = %f, y = %f, z = %f\n\r", my_gyro.x, my_gyro.y, my_gyro.z);
	        printf("Accel: x = %f, y = %f, z = %f\n\r", my_accel.x, my_accel.y, my_accel.z);
	        printf("Mag: x = %f, y = %f, z = %f\n\r", my_mag.x, my_mag.y, my_mag.z);
	      //  icm20948_sleep();

	        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	       	         	         	        // PARTIE Transmission Accéléromètre//
	      	       	         	         	        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//


	      	       	          			   // %%% transmission gyro%%%

	      	       	          	           snprintf(buf2, sizeof(buf2), "G %.2f ;%.2f ;%.2f ;\n",  my_gyro.x, my_gyro.y, my_gyro.z);
	      	       	          	           HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON

	      	       	          	           HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets
	      	       	          	     // HAL_Delay(1000);

	      	       	          	    // %%% transmission mag %%%
	      	       	          	   	       	          	           snprintf(buf2, sizeof(buf2), "M %.2f ;%.2f ;%.2f ;\n", my_mag.x,  my_mag.y, my_mag.z);
	      	       	          	   	       	          			  // HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON

	      	       	          	   	       	          			   HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets



	      	       	     	      	       	          	 //     HAL_Delay(1000);
	      	       	          	   	       	             /// TRANSMISSION DU NUMERO DE PAQUET et accélération///


	      	       	          	   	       	          HAL_Delay(3000);
	      	       	          	   	       	       	      	         snprintf(buf2, sizeof(buf2), "NumPaquet : %6d; A %.2f ;%.2f ;%.2f ;\n",capture_count++, my_accel.x,  my_accel.y, my_accel.z);
	      	       	          	   	       	       	      	       	      			   HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON


	      	       	          	   	       	       	      	       	      			   HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets

	      	       	          	   	       	       	      	         HAL_Delay(3000);
	      	       	          	   	       	       	      	       	 snprintf(buf2, sizeof(buf2), "%03d.%02d°C ;%d.%02d hPa ; %03d.%02d°C ;%d.%02d hPa ;\n", finaltemp_ref / 100, finaltemp_ref % 100, finalpressure_ref/100,finalpressure_ref%100,finaltemp_N / 100, finaltemp_N % 100, finalpressure_N/100,finalpressure_N%100);
	      	       	          	   	       	       	      	       		      			   HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON
	      	       	          	   	       	       	      	       		      			   // Cas où la chaîne KiwiRX est non vide> XON envoyé
	      	       	          	   	       	       	      	       		      			    HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets



	      	        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	        // PARTIE BME//
	      	        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	        BME280_GET_RAW_VALS_ref();
	      	           BME280_CALC_FINAL_VALS_ref();
	      	           printf("REF, Temperature: %d.%02d°C, Pressure: %d.%02d hPa\n\r",
	      	                   finaltemp_ref / 100, finaltemp_ref % 100, finalpressure_ref/100,finalpressure_ref%100);
	      	           HAL_Delay(1000); // A conserver, le capteur génère des erreurs autrement

	      	           // Lecture des données pour le capteur EXT, extérieur de la nacelle
	      	           BME280_GET_RAW_VALS_N();
	      	           BME280_CALC_FINAL_VALS_N();
	      	           printf("NORTH, Temperature: %d.%02d°C, Pressure: %d.%02d hPa\n\r",
	      	                   finaltemp_N / 100, finaltemp_N % 100, finalpressure_N/100,finalpressure_N%100);
	      	         HAL_Delay(1000);
	      	           // Lecture des données pour le baromètre - intérieur de la nacelle

	      	           int32_t pressure = Barometer_getPressure_E(true);
	      	           int32_t temp = Barometer_getTemp_E(true);


	      	           printf("EST, Temperature: %d.%02d°C, Pressure: %d.%02d hPa\n\r",
	      	                   temp/100, temp%100,pressure/100, pressure%100);

	      	           HAL_Delay(1000);
	      	         //  char filename[20];

	      	          // HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON




	      	           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	         	        // PARTIE Transmission série Kikiwi//
	      	         	        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
	      	           // %%% transmission capteur ref %%%
	      	           snprintf(buf2, sizeof(buf2), "%03d.%02d°C ;%d.%02d hPa ; %03d.%02d°C ;%d.%02d hPa ;\n", finaltemp_ref / 100, finaltemp_ref % 100, finalpressure_ref/100,finalpressure_ref%100,finaltemp_N / 100, finaltemp_N % 100, finalpressure_N/100,finalpressure_N%100);
	      			   HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON
	      			   // Cas où la chaîne KiwiRX est non vide> XON envoyé
	      			    HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets



	      			   // %%% transmission capteur nord %%%

	      	           snprintf(buf2, sizeof(buf2), "%d.%02drH ;%d.%02drH ;%03d.%02d°C ;%d.%02d hPa ;\n", final_humidity_ref/1024, final_humidity_ref%1024, final_humidity_N/1024, final_humidity_N%1024, temp/100, temp%100,pressure/100, pressure%100);
	      	           HAL_UART_Receive(&huart1, &received_char, 1, HAL_MAX_DELAY); // On attend  que la Kikiwi envoie XON
	      	           // Cas où la chaîne KiwiRX est non vide> XON envoyé
	      	           HAL_UART_Transmit(&huart1, (uint8_t*)buf2, strlen(buf2), HAL_MAX_DELAY); // transmission BME> Kikiwi au total 31 + 1(caractère \n ) octets




	      			    HAL_Delay(2680);

    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Infinite loop code could go here if needed */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA4 PA5 PA6
                           PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
