#ifndef SETUP_DOT_H
#define SETUP_DOT_H

#define GPIO_OUTPUT_IO_0    4
//#define GPIO_OUTPUT_IO_1    19
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0))// | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0     17
#define GPIO_INPUT_IO_1     18
#define GPIO_INPUT_IO_2     19
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1) | (1ULL<<GPIO_INPUT_IO_2))
#define ESP_INTR_FLAG_DEFAULT 0


void setup(void);

#endif