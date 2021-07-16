#include "Flow3.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"

#define GPIO_INPUT 5
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT) 
#define ESP_INTR_FLAG_DEFAULT 0
#define DEBOUNCETIME 25

volatile int Flow1cnt = 0;
volatile int lastState;
//volatile uint32_t debounceTimeout = 0;
unsigned long lastDetection = 0;
int stopper = 0;

// For setting up critical sections (enableinterrupts and disableinterrupts not available)
// used to disable and interrupt interrupts



//Signal between Pulse interrupt and pulse handling task
SemaphoreHandle_t xSemaphore = NULL;


// interrupt service routine, called when the falling edge is detected
static void IRAM_ATTR gpio_isr_handler(void* arg)
{ 
    xSemaphoreGiveFromISR(xSemaphore, NULL);
    //notify increment task
}

//Task that will react to faLLing edges 

static void gpio_task_inc(void* arg)
{
    for(;;) {
         // wait for the notification from the ISR
        if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {

             if((xTaskGetTickCount() - lastDetection) > DEBOUNCETIME){
                Flow1cnt++;
                //printf("GPIO_INPUT %d\n", buttonState);
                printf("Pulse count: %d \n", Flow1cnt);
                lastDetection = xTaskGetTickCount();
             }
            
        }
    }
}





int Flow1(char size)
{

    // create the binary semaphore
    xSemaphore = xSemaphoreCreateBinary();

    // set the correct direction
    gpio_set_direction(GPIO_INPUT, GPIO_MODE_INPUT);

    //Enable pull up resistor
    // gpio_pullup_en(GPIO_INPUT);
    //Disable pull up resistor
    gpio_pullup_dis(GPIO_INPUT);

    // enable interrupt on falling (1->0) edge for button pin
    gpio_set_intr_type(GPIO_INPUT, GPIO_INTR_NEGEDGE);

    // start the task that will handle the pulse
    xTaskCreate(gpio_task_inc, "gpio_task_inc", 2048, NULL, 10, NULL);

    // install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
    // attach the interrupt service routine
    gpio_isr_handler_add(GPIO_INPUT, gpio_isr_handler, NULL);


    if (size == 'M'){
        stopper = 10;
    }

    while(1){

        if (Flow1cnt>= stopper){

            gpio_uninstall_isr_service();
            Flow1cnt = 0;
            return 1;
        }

        vTaskDelay(100 / portTICK_RATE_MS);

    }
   return 0; 
}