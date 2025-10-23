/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "ArduCAM.h"
#include "icm20948.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
axises my_gyro;
axises my_accel;
axises my_mag;

FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];
char bufferRxBuffer[100];
char bufferRx[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *data, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)data, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SD_Select(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // CS low
}

void SD_Deselect(void) {
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // CS high
}
void capture_and_store_image(uint8_t *buffer, uint32_t buffer_size);
void capture_and_store_video(void);
void setup_sd_card(void);
void capture_and_store_image1(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//uint32_t wbytes; /* File write counts */
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  /* Initialisation de la caméra */
      ArduCAM_Init(OV2640); // Initialisation de la caméra OV2640

      set_format(JPEG);      // Configuration pour capturer en JPEG

      OV2640_set_JPEG_size(OV2640_1600x1200); // Configuration de la taille d'image
      HAL_Delay(1000);
      setup_sd_card();
      uint8_t *buffer = (uint8_t *)malloc(65536); // Augmentation du tampon à 8 Ko
          if (buffer == NULL) {
              printf("Erreur d'allocation mémoire.\n");
              Error_Handler();
          }
          int capture_count = 0;

          HAL_Delay(1000);

      //icm20948_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (capture_count < 100) {
	              capture_and_store_image(buffer, 65536);
	              capture_count++;
	          } else {
	              // Libérer le tampon et reformater la carte SD
	              free(buffer);
	              setup_sd_card();

	              // Réallouer le tampon
	              buffer = (uint8_t *)malloc(65536);
	              if (buffer == NULL) {
	                  printf("Erreur d'allocation mémoire.\n");
	                  Error_Handler();
	              }


	              capture_count = 0; // Réinitialiser le compteur de captures
	            }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void capture_and_store_image1(void) {
    /* Démarrage de la capture */
    start_capture(); // Démarrer la capture

    // Attente de la fin de la capture
    while (!(read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
        printf("Capture en cours...\n");
        HAL_Delay(100);
    }

    // Lire la taille de l'image dans la FIFO
    uint32_t length = read_fifo_length();
    printf("Taille de l'image : %lu octets\n", length);

    if (length == 0 || length >= 0x07FFFFF) {
        printf("Erreur : Taille d'image invalide.\n");
        return;
    }

    /* Montage de la carte SD */
    FRESULT fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        printf("Erreur lors du montage de la carte SD : %d\n", fres);
        Error_Handler();
    }

    /* Trouver un nom de fichier unique */
    char filename[32];
    uint16_t file_index = 1;
    while (file_index < 10000) { // Limite à 9999 fichiers
        snprintf(filename, sizeof(filename), "IMG%04d.JPG", file_index); // Générer le nom du fichier
        fres = f_open(&fil, filename, FA_CREATE_NEW | FA_WRITE);
        if (fres == FR_OK) {
            // Fichier créé avec succès
            break;
        } else if (fres == FR_EXIST) {
            // Fichier existe déjà, essayer un autre
            file_index++;
        } else {
            printf("Erreur lors de l'ouverture du fichier : %d\n", fres);
            Error_Handler();
        }
    }

    if (file_index >= 10000) {
        printf("Erreur : Limite de fichiers atteinte.\n");
        return;
    }

    printf("Fichier sélectionné : %s\n", filename);

    /* Préparation pour lecture en mode burst */
    set_fifo_burst();

    printf("Écriture de l'image sur la carte SD...\n");

    // Lecture des données de la FIFO et écriture sur la carte SD
    uint8_t buffer[512]; // Tampon pour les blocs de données
    uint32_t bytes_read = 0;
    while (bytes_read < length) {
        uint32_t chunk_size = (length - bytes_read > sizeof(buffer)) ? sizeof(buffer) : (length - bytes_read);

        // Lire en mode burst directement dans le tampon
        read_fifo_burst(buffer, chunk_size);

        // Écriture du tampon sur la carte SD
        UINT bytes_written;
        fres = f_write(&fil, buffer, chunk_size, &bytes_written);
        if (fres != FR_OK || bytes_written != chunk_size) {
            printf("Erreur lors de l'écriture sur la carte SD : %d\n", fres);
            Error_Handler();
        }

        bytes_read += chunk_size;
    }

    printf("Image écrite avec succès sur la carte SD.\n");

    /* Fermeture du fichier */
    fres = f_close(&fil);
    if (fres != FR_OK) {
        printf("Erreur lors de la fermeture du fichier : %d\n", fres);
        Error_Handler();
    }

    /* Démontage de la carte SD */
    fres = f_mount(NULL, "", 1);
    if (fres != FR_OK) {
        printf("Erreur lors du démontage de la carte SD : %d\n", fres);
        Error_Handler();
    }

    /* Nettoyage de la FIFO */
    flush_fifo();
    clear_fifo_flag();

    HAL_Delay(1000); // Petite pause pour éviter des conflits avec les prochaines captures
    printf("Capture et stockage terminés avec succès.\n");
}



void setup_sd_card(void) {
    SD_Select();
    FRESULT fres = f_mount(&fs, "", 0);
    if (fres != FR_OK) {
        SD_Deselect();
        printf("Erreur lors du montage de la carte SD.\n");
        Error_Handler();
    }
}


void capture_and_store_image(uint8_t *buffer, uint32_t buffer_size) {
    static uint32_t image_counter = 0;

    /* Démarrage de la capture */
    start_capture();
 HAL_Delay(1000);
    // Lecture de la taille de l'image dans la FIFO
    uint32_t length = read_fifo_length();
    if (length == 0) {
    	 printf("Taille est 0\n");
    	          }
    if (length == 0 || length >= 0x07FFFFF) {
        printf("Erreur : Taille d'image invalide.\n");
        return;
    }



    /* Génération d'un nom de fichier unique */
    char filename[20];
    snprintf(filename, sizeof(filename), "IMG%04lu.JPG", image_counter++);

    /* Création d'un fichier pour écrire l'image */
    FRESULT fres = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        printf("Erreur lors de l'ouverture du fichier %s.\n", filename);
        Error_Handler();
    }

    /* Préparation pour lecture en mode burst */
    set_fifo_burst();

    uint32_t bytes_read = 0;

    while (bytes_read < length) {
        uint32_t chunk_size = (length - bytes_read > buffer_size) ? buffer_size : (length - bytes_read);

        // Lecture en mode burst directement dans le tampon
        read_fifo_burst(buffer, chunk_size);

        // Écriture du tampon sur la carte SD
        UINT bytes_written;
        fres = f_write(&fil, buffer, chunk_size, &bytes_written);

        if (fres != FR_OK || bytes_written != chunk_size) {
            printf("Erreur lors de l'écriture sur la carte SD.\n");
            Error_Handler();
        }

        bytes_read += chunk_size;
        HAL_Delay(5);
    }

    /* Fermeture du fichier */
    fres = f_close(&fil);
    if (fres != FR_OK) {
        printf("Erreur lors de la fermeture du fichier %s.\n", filename);
        Error_Handler();
    }

    flush_fifo();
    clear_fifo_flag();

    printf("Image %s écrite avec succès.\n", filename);
}




void capture_and_store_video(void) {
    /* Montage de la carte SD */
    SD_Select();
    FRESULT fres = f_mount(&fs, "", 0);
    SD_Deselect();
    if (fres != FR_OK) {
        printf("Erreur lors du montage de la carte SD.\n");
        Error_Handler();
    }

    /* Création d'un fichier vidéo MJPEG */
    char video_filename[20];
    snprintf(video_filename, sizeof(video_filename), "VID001.MJP");

    SD_Select();
    fres = f_open(&fil, video_filename, FA_CREATE_ALWAYS | FA_WRITE);
    SD_Deselect();
    if (fres != FR_OK) {
        printf("Erreur lors de l'ouverture du fichier : %s\n", video_filename);
        Error_Handler();
    }

    printf("Démarrage de l'enregistrement vidéo...\n");

    /* Variables pour la capture */
    uint8_t buffer[512]; // Tampon pour lecture en blocs
    uint32_t frame_count = 0;
    uint32_t max_frames = 100; // Nombre maximum d'images à capturer
    uint32_t frame_delay = 10; // Délai entre les images (ms)
    UINT bytes_written;        // Variable pour stocker le nombre d'octets écrits

    while (frame_count < max_frames) {
        printf("Capture de l'image %lu...\n", frame_count + 1);

        /* Démarrage de la capture */
        start_capture();

        // Attente de la fin de la capture
        while (!(read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
            HAL_Delay(10);
        }

        // Lire la taille de l'image dans la FIFO
        uint32_t length = read_fifo_length();
        if (length == 0 || length >= 0x07FFFFF) {
            printf("Erreur : Taille d'image invalide.\n");
            flush_fifo();
            clear_fifo_flag();
            continue;
        }

        /* Préparation pour lecture en mode burst */
        set_fifo_burst();

        // Lecture des données de la FIFO et écriture dans le fichier MJPEG
        uint32_t bytes_read = 0;
        while (bytes_read < length) {
            uint32_t chunk_size = (length - bytes_read > sizeof(buffer)) ? sizeof(buffer) : (length - bytes_read);

            for (uint32_t i = 0; i < chunk_size; i++) {
                buffer[i] = read_fifo(); // Lire un octet de la FIFO
            }

            // Écriture du tampon dans le fichier
            SD_Select();
            fres = f_write(&fil, buffer, chunk_size, &bytes_written);
            SD_Deselect();

            if (fres != FR_OK || bytes_written != chunk_size) {
                printf("Erreur lors de l'écriture de l'image %lu. Code d'erreur : %d\n", frame_count + 1, fres);
                flush_fifo();
                clear_fifo_flag();
                Error_Handler();
            }

            bytes_read += chunk_size;
        }

        /* Ajout d'un marqueur de fin de frame (facultatif) */
        const uint8_t end_marker[2] = {0xFF, 0xD9}; // Marqueur de fin JPEG
        SD_Select();
        fres = f_write(&fil, end_marker, sizeof(end_marker), &bytes_written);
        SD_Deselect();
        if (fres != FR_OK || bytes_written != sizeof(end_marker)) {
            printf("Erreur lors de l'ajout du marqueur de fin à l'image %lu. Code d'erreur : %d\n", frame_count + 1, fres);
            flush_fifo();
            clear_fifo_flag();
            Error_Handler();
        }

        printf("Image %lu écrite avec succès.\n", frame_count + 1);

        /* Nettoyage de la FIFO */
        flush_fifo();
        clear_fifo_flag();

        frame_count++;
        HAL_Delay(frame_delay); // Délai entre les captures
    }

    printf("Enregistrement vidéo terminé.\n");

    /* Fermeture du fichier */
    SD_Select();
    if (f_close(&fil) != FR_OK) {
        printf("Erreur lors de la fermeture du fichier vidéo.\n");
        Error_Handler();
    }
    SD_Deselect();

    /* Démontage de la carte SD */
    SD_Select();
    if (f_mount(NULL, "", 1) != FR_OK) {
        printf("Erreur lors du démontage de la carte SD.\n");
        Error_Handler();
    }
    SD_Deselect();

    HAL_Delay(8000);
}

/* Fonction pour créer un fichier texte */
FRESULT create_text_file(const char *filename) {
    FIL fil;        // Objet fichier
    FRESULT fres;   // Code de retour FatFs

    // Monter le système de fichiers
    fres = f_mount(&fs, "", 0);
    if (fres != FR_OK) {
        printf("Erreur lors du montage du système de fichiers.\n");
        return fres;
    }

    // Créer un fichier vide
    fres = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (fres != FR_OK) {
        printf("Erreur lors de la création du fichier %s.\n", filename);
        return fres;
    }

    // Fermer le fichier
    fres = f_close(&fil);
    if (fres != FR_OK) {
        printf("Erreur lors de la fermeture du fichier %s.\n", filename);
    }

    printf("Fichier %s créé avec succès.\n", filename);
    return fres;
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
