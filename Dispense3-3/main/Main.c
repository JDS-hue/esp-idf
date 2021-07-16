#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "Flow1.h"
#include "I2C_PE.h"

//idf.py -p COM5 clean flash monitor


//INPUT BUTTON AND OUTPUT PIN DEFINTIONS
#define GPIO_INPUT_IO_B     21
#define GPIO_INPUT_PIN_SELB  (1ULL<<GPIO_INPUT_IO_B)
#define GPIO_OUTPUT_IO_B     18
#define GPIO_OUTPUT_PIN_SELB  (1ULL<<GPIO_OUTPUT_IO_B)
#define ESP_INTR_FLAG_DEFAULT 0

//I2C DEFINITIONS
#define I2C_MASTER_SDA_IO 23
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 
#define PEXPANDER_ADDR 0x20

#define EXAMPLE_I2C_ACK_CHECK_EN			0x1
#define EXAMPLE_I2C_ACK_CHECK_DIS			0x0
#define EXAMPLE_I2C_ACK_VAL					0x0
#define EXAMPLE_I2C_NACK_VAL				0x1


//I2C INTERRUPT DEFNITIONS
#define GPIO_INPUT_I2C 4
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT) 
#define ESP_INTR_FLAG_DEFAULT 0


struct product
{
    int type;
    int size;
    int price;
    int time;
};

struct product hold;
int SelFlag = 0;


uint8_t data_h = 0;
int statemain = 0;
int bin1 = 0;
int bin2 = 0;
int binS = 0;
int line = 0;



//Signal between I2X interrupt and I2C handling task
SemaphoreHandle_t xSemaphoreI2C = NULL;

//12C port number
const i2c_port_t I2C_PORT = I2C_NUM_1;

//
bool Initialize = false;



// interrupt service routine, called when the falling edge is detected
static void IRAM_ATTR i2c_isr_handler(void* arg)
{ 
    xSemaphoreGiveFromISR(xSemaphoreI2C, NULL);
    //notify i2c read task
}


static void i2c_read_task(void* arg)
{
    for(;;) {
         // wait for the notification from the ISR
        if(xSemaphoreTake(xSemaphoreI2C,portMAX_DELAY) == pdTRUE) {

             //read all port expander values 

            int val = readallPE();

            if ((val == 11)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 1;
                bin1 = 83;
                bin2 = 115;
                line = 1;
            }
            if ((val == 12)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 2;
                bin1 = 147;
                bin2 = 179;
                line = 1;
            }
            if ((val == 13)&&(SelFlag == 1)&&( line == 1)){
                statemain = 2;
                SelFlag = 3;
                binS = 127;
                line = 1;
            }
             if ((val == 13)&&(SelFlag == 2)&&( line == 1)){
                statemain = 2;
                SelFlag = 3;
                binS = 191;
                line = 1;
            }
            
            if ((val == 21)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 1;
                bin1 = 83;
                bin2 = 115;
                line = 2;
            }
            if ((val == 22)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 2;
                bin1 = 147;
                bin2 = 179;
                line = 2;
            }
            if ((val == 23)&&(SelFlag == 1)&&( line == 2)){
                statemain = 2;
                SelFlag = 2;
                binS = 127;
               
            }
            if ((val == 23)&&(SelFlag == 2)&&( line == 2)){
                statemain = 2;
                SelFlag = 3;
                binS = 191;

            }
            if ((val == 31)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 1;
                bin1 = 83;
                bin2 = 115;
                 line = 3;
            }
            if ((val == 32)&&(SelFlag == 0)){
                statemain = 1;
                SelFlag = 1;
                bin1 = 147;
                bin2 = 179;
                line = 3;
            }
            if ((val == 33)&&(SelFlag == 1)&&( line == 3)){
                statemain = 2;
                SelFlag = 2;
                binS = 127;
            }
             if ((val == 33)&&(SelFlag == 2)&&( line == 3)){
                statemain = 2;
                SelFlag = 3;
                binS = 191;
             }
            

             //set flag for main loop to run correct function

             //run flash button function for correct line - within function set flag for line 1, 2 and 3 if correct start is pressed then run line for how ever long
            
        }
    }
}

void static printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;
  
    
    for (i = size-1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}



void static InitializeGPIO(void){
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SELB;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SELB;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


}

void InitializeI2C(void){

    const i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER, // 
      .sda_io_num = I2C_MASTER_SDA_IO, //SDA GPIO
      .sda_pullup_en = GPIO_PULLUP_ENABLE,    //Enable Input
      .scl_io_num = I2C_MASTER_SCL_IO,         //Scl GPIO
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ, 

  };

    i2c_param_config(I2C_PORT, &i2c_config);
    i2c_driver_install(I2C_PORT, i2c_config.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    esp_err_t i2c_ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //----- WRITE BYTES -----
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0b01000000, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00010011, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_PORT, cmd, (1000 / portTICK_RATE_MS)); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);
    if (i2c_ret == ESP_OK)
    {
        printf("I2C Write OK\n");
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, 0b01000001, EXAMPLE_I2C_ACK_CHECK_EN);
        i2c_master_read_byte(cmd, &data_h, EXAMPLE_I2C_NACK_VAL);

        i2c_master_stop(cmd);
        //Send queued commands
        i2c_ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
        i2c_cmd_link_delete(cmd);
    }
    else
    {
        printf("I2C FAILED\n");
    }

    uint8_t val = data_h << 2;
    


    printf("data_h:%d\n", data_h);
    printf("data_h:%x\n", data_h);
    printf("data_h:\n");
    printBits(sizeof(data_h),&data_h);
    printBits(sizeof(val),&val);

    if(data_h  == 238){
        printf("success");
    }else{
        printf("failure");
    }
    


}

void I2CIntInit(void){

    xSemaphoreI2C = xSemaphoreCreateBinary();

    // set the correct direction
    gpio_set_direction(GPIO_INPUT_I2C, GPIO_MODE_INPUT);

    //Enable pull up resistor
    // gpio_pullup_en(GPIO_INPUT);
    //Disable pull up resistor
    gpio_pullup_dis(GPIO_INPUT_I2C);

    // enable interrupt on falling (1->0) edge for button pin
    gpio_set_intr_type(GPIO_INPUT_I2C, GPIO_INTR_NEGEDGE);

    // start the task that will handle the pulse
    xTaskCreate(i2c_read_task, "i2c_read_task", 2048, NULL, 10, NULL);

    // install ISR service with default configuration
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
 
    // attach the interrupt service routine
    gpio_isr_handler_add(GPIO_INPUT_I2C, i2c_isr_handler, NULL);
}





void app_main(void) 
{
    InitializeGPIO();
    InitializeI2C();
    I2CIntInit();
    printf("GPIO's Initialized\n");
    printf("State: %d\n", statemain);
    gpio_set_level(GPIO_OUTPUT_IO_B, 1);


    while (1)
    {

         if (statemain == 0)
         { //Idle state where processor waits for inputs from buttons, keypad, firmware update

            //  if (xSemaphoreTake(xSemaphoreI2C,portMAX_DELAY) == pdTRUE)
            //  {

            //      //read all port expander values 

    

            //  //run flash button function for correct line - within function set flag for line 1, 2 and 3 if correct start is pressed then run line for how ever long
            //      statemain = 3;
            //   printf("State: %d\n", statemain);
            //      printf("Button pressed\n");
                
            //  }
         }

          if (statemain == 1){

           
            int starttime = xTaskGetTickCount();

             flashandwait(line,starttime,bin1,bin2);

          }

         if (statemain == 2)
         { 
             if (Flow1('M') == 1)
             {
                 gpio_set_level(GPIO_OUTPUT_IO_B, 0);
                 printf("Done counting\n");
                 statemain = 0;

             }
         }
             
        
       vTaskDelay(100/portTICK_PERIOD_MS);
       //vTaskDelete(NULL);
    }
}
