#include "I2C_PE.h"
#include "driver/i2c.h"

// #define SAMPLE_PERIOD_MS		200

// #define I2C_SCL_IO				22	//19               /*!< gpio number for I2C master clock */
// #define I2C_SDA_IO				23	//18               /*!< gpio number for I2C master data  */
// #define I2C_FREQ_HZ				100000           /*!< I2C master clock frequency */
// #define I2C_PORT_NUM			I2C_NUM_1        /*!< I2C port number for master dev */
// #define I2C_TX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */
// #define I2C_RX_BUF_DISABLE  	0                /*!< I2C master do not need buffer */

// // I2C common protocol defines
// #define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
// #define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
// #define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
// #define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
// #define ACK_VAL                            0x0              /*!< I2C ack value */
// #define NACK_VAL                           0x1              /*!< I2C nack value */

//idf.py -p COM5 clean flash monitor
#define EXAMPLE_I2C_ACK_CHECK_EN			0x1
#define EXAMPLE_I2C_ACK_CHECK_DIS			0x0
#define EXAMPLE_I2C_ACK_VAL					0x0
#define EXAMPLE_I2C_NACK_VAL				0x1
#define P1_ADDR 0x20
#define P2_ADDR 0x21
#define P3_ADDR 0x22
#define P4_ADDR 0x23

esp_err_t i2c_ret = ESP_OK;

//I2C data holds
uint8_t data_p1 = 0;
uint8_t data_p2 = 0;
uint8_t data_p3 = 0;
uint8_t data_p4 = 0;

extern int SelFlag;
extern int statemain; 



int static PEdecode(uint8_t data)
{

    if (data == 1){  //400
        return 1;
    }else  if (data == 2){  //750
        return 2;
    }else if (data == 16){  //start
        return 3; 
    }else {
        return 4;  //error
    }
    
}

char static keydecode(uint8_t data)
{
    if (data == 238){
        return '1';
    }else  if (data == 222){
        return '2';
    }else if (data == 190){
        return '3';
    }else if (data == 237){
        return '4';
    }else if (data == 221){
        return '5';
    }else if (data == 189){
        return '6';
    }else if (data == 235){
        return '7';
    }else if (data == 219){
        return '8';
    }else if (data == 187){
        return '9';
    }else if (data == 231){
        return '#';
    }else if (data == 215){
        return '0';
    }else if (data == 183){
        return '*';
    }
    
    
    return 'e';
}

int readallPE(void)
{

    ///////PE1//////////
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //----- WRITE BYTES -----
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, P1_ADDR, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00010011, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS)); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (P1_ADDR << 1 | 1), EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_p1, EXAMPLE_I2C_NACK_VAL);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    ///////PE2//////////
    
    //----- WRITE BYTES -----
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, P2_ADDR, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00010011, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS)); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (P2_ADDR << 1 | 1), EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_p2, EXAMPLE_I2C_NACK_VAL);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    ///////PE3//////////

    //----- WRITE BYTES -----
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, P3_ADDR, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00010011, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS)); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (P3_ADDR << 1 | 1), EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_p3, EXAMPLE_I2C_NACK_VAL);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    ///////PE4//////////

    //----- WRITE BYTES -----
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, P4_ADDR, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00010011, EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, (1000 / portTICK_RATE_MS)); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (P4_ADDR << 1 | 1), EXAMPLE_I2C_ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_p4, EXAMPLE_I2C_NACK_VAL);
    i2c_master_stop(cmd);
    //Send queued commands
    i2c_ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS); //"(# / portTICK_RATE_MS)"=maximum wait time. This task will be blocked until all the commands have been sent (not thread-safe - if you want to use one I2C port in different tasks you need to take care of multi-thread issues)
    i2c_cmd_link_delete(cmd);


    if (PEdecode(data_p1) == 1 ){
        return 11; //line 1, 400
    }else if(PEdecode(data_p1) == 2 ){
        return 12; //line 1, 750
    }else if(PEdecode(data_p1) == 3 ){
        return 13; //line 1, start
    }else if (PEdecode(data_p2) == 1 ){
        return 21; //line 1, 400
    }else if(PEdecode(data_p2) == 2 ){
        return 22; //line 1, 750
    }else if(PEdecode(data_p2) == 3 ){
        return 23; //line 1, start
    }else if (PEdecode(data_p3) == 1 ){
        return 31; //line 1, 400
    }else if(PEdecode(data_p3) == 2 ){
        return 32; //line 1, 750
    }else if(PEdecode(data_p3) == 3 ){
        return 33; //line 1, start
    }else  if (keydecode(data_p4) == '0'){
        return 40;
    }else  if (keydecode(data_p4) == '1'){
        return 41;
    }else  if (keydecode(data_p4) == '2'){
        return 42;
    }else  if (keydecode(data_p4) == '3'){
        return 43;
    }else  if (keydecode(data_p4) == '4'){
        return 44;
    }else  if (keydecode(data_p4) == '5'){
        return 45;
    }else  if (keydecode(data_p4) == '6'){
        return 46;
    }else  if (keydecode(data_p4) == '7'){
        return 47;
    }else  if (keydecode(data_p4) == '8'){
        return 48;
    }else  if (keydecode(data_p4) == '9'){
        return 49;
    }else  if (keydecode(data_p4) == '#'){
        return 50;
    }else  if (keydecode(data_p4) == '*'){
        return 51;
    }else {
        return 52;
    }

    return 0;
}

void flashandwait(int line, int starttime,int bin1,int bin2)
{

    if (line == 1){
        int add = P1_ADDR;
    }else if (line == 2){
        int add = P2_ADDR;
    }else if (line == 3){
        int add = P3_ADDR;
    }


        while ((((xTaskGetTickCount() - starttime) / portTICK_PERIOD_MS) < 10000) && (statemain == 1))
        {
            int p_t = (10000 - ((xTaskGetTickCount() - starttime) / portTICK_PERIOD_MS))

            if ((p_t/500)% 2 == 0){
                //set start led high
            }

            if ((p_t/500)% 2 == 1){
                //set start led low
            }



            if (((xTaskGetTickCount() - starttime) / portTICK_PERIOD_MS)> 10000)
                {
                    SelFlag = 0;
                    statemain = 0;
                }
        }
}
    