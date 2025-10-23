	/*
	 * BME280.c
	 *
	 *  Created on: Dec 11, 2024
	 *      Author: david
	 */
	#include "main.h"
	#include "BME.h"
	#include "stm32l1xx_hal.h"

	extern SPI_HandleTypeDef hspi2;

	extern UART_HandleTypeDef huart2;

	uint8_t tempread_ref[8];
	uint8_t config_ref[2];
	uint8_t comp_ref[32];

	uint8_t tempread_N[8];
	uint8_t config_N[2];
	uint8_t comp_N[32];

	int finaltemp_ref = 0;
	int finaltemp_N = 0;

	uint32_t finalpressure_ref = 0;
	uint32_t final_humidity_ref = 0;

	uint32_t finalpressure_N = 0;
	uint32_t final_humidity_N = 0;

	unsigned char dig_H1_ref, dig_H3_ref;
	signed char dig_H6_ref;
	unsigned short dig_T1_ref, dig_P1_ref;
	signed short dig_T2_ref, dig_T3_ref, dig_P2_ref, dig_P3_ref, dig_P4_ref, dig_P5_ref, dig_P6_ref, dig_P7_ref, dig_P8_ref, dig_P9_ref, dig_H2_ref, dig_H4_ref, dig_H5_ref;

	unsigned char dig_H1_N, dig_H3_N;
	signed char dig_H6_N;
	unsigned short dig_T1_N, dig_P1_N;
	signed short dig_T2_N, dig_T3_N, dig_P2_N, dig_P3_N, dig_P4_N, dig_P5_N, dig_P6_N, dig_P7_N, dig_P8_N, dig_P9_N, dig_H2_N, dig_H4_N, dig_H5_N;

	volatile int temperature_raw_ref, pressure_raw_ref, humidity_raw_ref = 0;
	volatile int temperature_raw_N, pressure_raw_N, humidity_raw_N = 0;

	void BME280_CONFIG_SETUP_ref(void){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		config_ref[0] = CTRLMEASREG;
		config_ref[1] = CTRLMEASVAL;
		HAL_SPI_Transmit(&hspi2, config_ref, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		config_ref[0] = CONFIGREG;
		config_ref[1] = CONFIGVAL;
		HAL_SPI_Transmit(&hspi2, config_ref, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		config_ref[0] = CTRLHUMREG;
		config_ref[1] = CTRLHUMVAL;
		HAL_SPI_Transmit(&hspi2, config_ref, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);
	}
	void BME280_GET_RAW_VALS_ref(void){
		BME280_CONFIG_SETUP_ref();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		config_ref[0] = RAWREAD;
		HAL_SPI_Transmit(&hspi2, config_ref, 1, 10); //GET ID
		HAL_SPI_Receive(&hspi2, tempread_ref, 8, 10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

		temperature_raw_ref =(tempread_ref[3]<<12)+(tempread_ref[4]<<4)+(tempread_ref[5]>>4);
		pressure_raw_ref = (tempread_ref[0]<<12)+(tempread_ref[1]<<4)+(tempread_ref[2]>>4);
		humidity_raw_ref= (tempread_ref[6] << 8) + (tempread_ref[7]);
	}
	void BME280_GET_COMP_VALS_ref(void){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		config_ref[0] = COMPTEMPPRES;
		HAL_SPI_Transmit(&hspi2, config_ref, 1, 10);
		HAL_SPI_Receive(&hspi2, comp_ref, 24, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);

		dig_T1_ref = (comp_ref[0])+(comp_ref[1]<<8);
		dig_T2_ref = (comp_ref[2])+(comp_ref[3]<<8);
		dig_T3_ref = (comp_ref[4])+(comp_ref[5]<<8);
		dig_P1_ref = (comp_ref[6])+(comp_ref[7]<<8);
		dig_P2_ref = (comp_ref[8])+(comp_ref[9]<<8);
		dig_P3_ref = (comp_ref[10])+(comp_ref[11]<<8);
		dig_P4_ref = (comp_ref[12])+(comp_ref[13]<<8);
		dig_P5_ref = (comp_ref[14])+(comp_ref[15]<<8);
		dig_P6_ref = (comp_ref[16])+(comp_ref[17]<<8);
		dig_P7_ref = (comp_ref[18])+(comp_ref[19]<<8);
		dig_P8_ref = (comp_ref[20])+(comp_ref[21]<<8);
		dig_P9_ref = (comp_ref[22])+(comp_ref[23]<<8);

		config_ref[0] = COMPHUMINIT;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		HAL_SPI_Transmit(&hspi2, config_ref, 1, 10);
		HAL_SPI_Receive(&hspi2, &comp_ref[24], 1, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);
		dig_H1_ref = comp_ref[24];

		config_ref[0] = COMPHUMREST;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //SET CS LOW
		HAL_SPI_Transmit(&hspi2, config_ref, 1, 10);
		HAL_SPI_Receive(&hspi2, &comp_ref[25], 7, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(10);
		dig_H2_ref = (comp_ref[25])+(comp_ref[26]<< 8);
		dig_H3_ref = comp_ref[27];
		dig_H4_ref = (comp_ref[28] << 4) +(comp_ref[29] & 0xF);
		dig_H5_ref = (comp_ref[29] & 0xF0) +(comp_ref[30]<< 4);
		dig_H6_ref = comp_ref[31];
	}



	void BME280_CALC_FINAL_VALS_ref(void){
		int var1, var2, t_fine;
		var1 = ((((temperature_raw_ref >> 3) - ((int32_t)dig_T1_ref << 1))) * ((int32_t)dig_T2_ref)) >> 11;
		var2 = (((((temperature_raw_ref >> 4) - ((int32_t)dig_T1_ref)) * ((temperature_raw_ref >> 4) - ((int32_t)dig_T1_ref))) >> 12) * ((int32_t)dig_T3_ref)) >> 14;
		t_fine = var1 + var2;
		finaltemp_ref = (t_fine * 5 + 128) >> 8;

		var1 = (((int)t_fine) >> 1) - 64000;
		var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int)dig_P6_ref);
		var2 = var2 + ((var1 * ((int)dig_P5_ref)) << 1);
		var2 = (var2 >> 2) + (((int) dig_P4_ref) << 16);
		var1 = (((dig_P3_ref * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int) dig_P2_ref) * var1) >> 1 )) >> 18;
		var1 = ((((32768 + var1)) * ((int)dig_P1_ref)) >> 15);
		if (var1 == 0)
		{
			finalpressure_ref = 0;
		}
		else{
			finalpressure_ref = (((uint32_t) (((int)1048576)-pressure_raw_ref) - (var2 >> 12))) * 3125;
			if (finalpressure_ref < 0x80000000){
				finalpressure_ref = (finalpressure_ref << 1) / (( uint32_t)var1);
			}
			else{
				finalpressure_ref = (finalpressure_ref / (uint32_t)var1) * 2;
			}
			var1 = (((int)dig_P9_ref) * ((int) ((( finalpressure_ref >> 3) * ( finalpressure_ref >> 3)) >> 13))) >> 12;
			var2 = (((int) (finalpressure_ref >> 2)) * ((int)dig_P8_ref)) >> 13;
			finalpressure_ref = ((uint32_t)((int)finalpressure_ref + ((var1 + var2 + dig_P7_ref) >> 4))); //kPA
		}

		var1 = (t_fine - ((int) 76800));
		var1 = (((((humidity_raw_ref << 14) - (((int) dig_H4_ref) << 20) - (((int)dig_H5_ref) * var1)) + ((int) 16384)) >> 15) * \
		(((((((var1 * ((int) dig_H6_ref)) >> 10) * (((var1 * ((int) dig_H3_ref)) >> 11) + ((int) 32768))) >> 10) + \
		((int) 2097152)) * ((int) dig_H2_ref) + 8192) >> 14));

		var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int)dig_H1_ref)) >> 4));
		var1 = (var1 < 0 ? 0 : var1);
		var1 = (var1 > 419430400 ? 419330400 : var1);
		final_humidity_ref = (var1 >> 12)/1024;
	}



	void BME280_CONFIG_SETUP_N(void){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		config_N[0] = CTRLMEASREG;
		config_N[1] = CTRLMEASVAL;
		HAL_SPI_Transmit(&hspi2, config_N, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		config_N[0] = CONFIGREG;
		config_N[1] = CONFIGVAL;
		HAL_SPI_Transmit(&hspi2, config_N, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		config_N[0] = CTRLHUMREG;
		config_N[1] = CTRLHUMVAL;
		HAL_SPI_Transmit(&hspi2, config_N, 2, 1000); //CONFIG
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);
	}
	void BME280_GET_RAW_VALS_N(void){
		BME280_CONFIG_SETUP_N();
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		config_N[0] = RAWREAD;
		HAL_SPI_Transmit(&hspi2, config_N, 1, 10); //GET ID
		HAL_SPI_Receive(&hspi2, tempread_N, 8, 10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

		temperature_raw_N =(tempread_N[3]<<12)+(tempread_N[4]<<4)+(tempread_N[5]>>4);
		pressure_raw_N = (tempread_N[0]<<12)+(tempread_N[1]<<4)+(tempread_N[2]>>4);
		humidity_raw_N = (tempread_N[6] << 8) + (tempread_N[7]);
	}
	void BME280_GET_COMP_VALS_N(void){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		config_N[0] = COMPTEMPPRES;
		HAL_SPI_Transmit(&hspi2, config_N, 1, 10);
		HAL_SPI_Receive(&hspi2, comp_N, 24, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);

		dig_T1_N = (comp_N[0])+(comp_N[1]<<8);
		dig_T2_N = (comp_N[2])+(comp_N[3]<<8);
		dig_T3_N = (comp_N[4])+(comp_N[5]<<8);
		dig_P1_N = (comp_N[6])+(comp_N[7]<<8);
		dig_P2_N = (comp_N[8])+(comp_N[9]<<8);
		dig_P3_N = (comp_N[10])+(comp_N[11]<<8);
		dig_P4_N = (comp_N[12])+(comp_N[13]<<8);
		dig_P5_N = (comp_N[14])+(comp_N[15]<<8);
		dig_P6_N = (comp_N[16])+(comp_N[17]<<8);
		dig_P7_N = (comp_N[18])+(comp_N[19]<<8);
		dig_P8_N = (comp_N[20])+(comp_N[21]<<8);
		dig_P9_N = (comp_N[22])+(comp_N[23]<<8);

		config_N[0] = COMPHUMINIT;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		HAL_SPI_Transmit(&hspi2, config_N, 1, 10);
		HAL_SPI_Receive(&hspi2, &comp_N[24], 1, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);
		dig_H1_N = comp_N[24];

		config_N[0] = COMPHUMREST;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //SET CS LOW
		HAL_SPI_Transmit(&hspi2, config_N, 1, 10);
		HAL_SPI_Receive(&hspi2, &comp_N[25], 7, 120);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(10);
		dig_H2_N = (comp_N[25])+(comp_N[26]<< 8);
		dig_H3_N = comp_N[27];
		dig_H4_N = (comp_N[28] << 4) +(comp_N[29] & 0xF);
		dig_H5_N = (comp_N[29] & 0xF0) +(comp_N[30]<< 4);
		dig_H6_N = comp_N[31];
	}



	void BME280_CALC_FINAL_VALS_N(void){
		int var1, var2, t_fine;
		var1 = ((((temperature_raw_N >> 3) - ((int32_t)dig_T1_N << 1))) * ((int32_t)dig_T2_N)) >> 11;
		var2 = (((((temperature_raw_N >> 4) - ((int32_t)dig_T1_N)) * ((temperature_raw_N >> 4) - ((int32_t)dig_T1_N))) >> 12) * ((int32_t)dig_T3_N)) >> 14;
		t_fine = var1 + var2;
		finaltemp_N = (t_fine * 5 + 128) >> 8;

		var1 = (((int)t_fine) >> 1) - 64000;
		var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((int)dig_P6_N);
		var2 = var2 + ((var1 * ((int)dig_P5_N)) << 1);
		var2 = (var2 >> 2) + (((int) dig_P4_N) << 16);
		var1 = (((dig_P3_N * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((int) dig_P2_N) * var1) >> 1 )) >> 18;
		var1 = ((((32768 + var1)) * ((int)dig_P1_N)) >> 15);
		if (var1 == 0)
		{
			finalpressure_N = 0;
		}
		else{
			finalpressure_N = (((uint32_t) (((int)1048576)-pressure_raw_N) - (var2 >> 12))) * 3125;
			if (finalpressure_N < 0x80000000){
				finalpressure_N = (finalpressure_N << 1) / (( uint32_t)var1);
			}
			else{
				finalpressure_N = (finalpressure_N / (uint32_t)var1) * 2;
			}
			var1 = (((int)dig_P9_N) * ((int) ((( finalpressure_N >> 3) * ( finalpressure_N >> 3)) >> 13))) >> 12;
			var2 = (((int) (finalpressure_N >> 2)) * ((int)dig_P8_N)) >> 13;
			finalpressure_N = ((uint32_t)((int)finalpressure_N + ((var1 + var2 + dig_P7_N) >> 4))); //kPA
		}

		var1 = (t_fine - ((int) 76800));
		var1 = (((((humidity_raw_N << 14) - (((int) dig_H4_N) << 20) - (((int)dig_H5_N) * var1)) + ((int) 16384)) >> 15) * \
		(((((((var1 * ((int) dig_H6_N)) >> 10) * (((var1 * ((int) dig_H3_N)) >> 11) + ((int) 32768))) >> 10) + \
		((int) 2097152)) * ((int) dig_H2_N) + 8192) >> 14));

		var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int)dig_H1_N)) >> 4));
		var1 = (var1 < 0 ? 0 : var1);
		var1 = (var1 > 419430400 ? 419330400 : var1);
		final_humidity_N = (var1 >> 12)/1024;
	}
