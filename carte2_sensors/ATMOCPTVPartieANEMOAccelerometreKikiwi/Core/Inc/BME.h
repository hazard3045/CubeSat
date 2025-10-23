#ifndef BME_H
#define BME_H

#define CTRLMEASREG 0x74
#define CTRLMEASVAL 0x25
#define CONFIGREG 0x75
#define CONFIGVAL 0xA0
#define CTRLHUMREG 0x72
#define CTRLHUMVAL 0x01
#define COMPTEMPPRES 0x88
#define COMPHUMINIT 0xA0
#define COMPHUMREST 0xE1
#define RAWREAD 0xF7

void BME280_CONFIG_SETUP_ref(void);
void BME280_GET_COMP_VALS_ref(void);
void BME280_GET_RAW_VALS_ref(void);
void BME280_CALC_FINAL_VALS_ref(void);

void BME280_CONFIG_SETUP_N(void);
void BME280_GET_COMP_VALS_N(void);
void BME280_GET_RAW_VALS_N(void);
void BME280_CALC_FINAL_VALS_N(void);

extern uint32_t finalpressure;
extern uint32_t final_humidity;
extern int finaltemp;
extern uint8_t tempread[8];
extern uint8_t config[2];
extern uint8_t comp[32];
#endif
