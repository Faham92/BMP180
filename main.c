#include <math.h>
#include "esp_system.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_sleep.h"



#define ACK_CHECK_EN    0x1     // I2C master will check ack from slave
#define ACK_CHECK_DIS   0x0     // I2C master will not check ack from slave
#define ACK_VAL         0x0     // I2C ack value
#define NACK_VAL        0x1     // I2C nack value

#define BMP180_ADDRESS 0x77     // I2C address of BMP180
#define BMP180_W       0xEE
#define BMP180_R       0xEF

#define SDA 21
#define SCL 22
 
#define BMP180_ULTRA_LOW_POWER  0
#define BMP180_STANDARD         1
#define BMP180_HIGH_RES         2
#define BMP180_ULTRA_HIGH_RES   3

#define BMP180_CAL_AC1          0xAA  // Calibration data (16 bits)
#define BMP180_CAL_AC2          0xAC  // Calibration data (16 bits)
#define BMP180_CAL_AC3          0xAE  // Calibration data (16 bits)
#define BMP180_CAL_AC4          0xB0  // Calibration data (16 bits)
#define BMP180_CAL_AC5          0xB2  // Calibration data (16 bits)
#define BMP180_CAL_AC6          0xB4  // Calibration data (16 bits)
#define BMP180_CAL_B1           0xB6  // Calibration data (16 bits)
#define BMP180_CAL_B2           0xB8  // Calibration data (16 bits)
#define BMP180_CAL_MB           0xBA  // Calibration data (16 bits)
#define BMP180_CAL_MC           0xBC  // Calibration data (16 bits)
#define BMP180_CAL_MD           0xBE  // Calibration data (16 bits)

#define BMP180_CONTROL             0xF4  // Control register
#define BMP180_DATA_TO_READ        0xF6  // Read results here
#define BMP180_READ_TEMP_CMD       0x2E  // Request temperature measurement
#define BMP180_READ_PRESSURE_CMD   0xB4  // Request pressure measurement

// Calibration parameters
static int16_t ac1;
static int16_t ac2;
static int16_t ac3;
static uint16_t ac4;
static uint16_t ac5;
static uint16_t ac6;
static int16_t b1;
static int16_t b2;
static int16_t mb;
static int16_t mc;
static int16_t md;
static uint8_t oversampling = BMP180_HIGH_RES  ; //configure le capteur en mode ultra low power


RTC_DATA_ATTR int bootCount = 0;

//déclaration des fonctions
static esp_err_t bmp180_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size);
static esp_err_t bmp180_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd);
static esp_err_t bmp180_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size);
esp_err_t bmp180_init(int pin_sda, int pin_scl);
static esp_err_t bmp180_read_uncompensated_temperature(int16_t* temp);
static void read_tempeature(float *value, int32_t);
static esp_err_t bmp180_calculate_b5(int32_t* b5);
static uint32_t bmp180_read_uncompensated_pressure(uint32_t* up);
static void read_pressure(int32_t *value, int32_t);
void print_wakeup_reason();





void app_main()
{   
    float temperature;
    int32_t pression,b5;
    esp_err_t test;
    int time = 40 ; 
    vTaskDelay(pdMS_TO_TICKS(10000)); // delai avant affichage des données et l'initialisation de l'i2c
    bmp180_init(SDA, SCL);
    print_wakeup_reason();
    esp_sleep_enable_timer_wakeup(time * 1000000); //temporisation en secondes (*1000000 car le temps est en micro seconde dans la fonction timer_wake_up)
        while(1)
        {
            bootCount++;
           test = bmp180_calculate_b5(&b5);
            read_tempeature(&temperature,b5);
            read_pressure(&pression,b5);
            ESP_LOGI("[MAIN]","La température est de %f °C",temperature);
            ESP_LOGI("[MAIN]","La pression est de %d hPa",pression);
            vTaskDelay(pdMS_TO_TICKS(2000));  
            if(bootCount %10 == 0)
             {
                ESP_LOGI("[MAIN]","Going to sleep now\r\n");
                esp_deep_sleep_start();
             }      
          
        } 
 }


static esp_err_t bmp180_master_write_slave(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();  //initialise commande i2c
    i2c_master_start(cmd);  
    i2c_master_write_byte(cmd, BMP180_W, ACK_CHECK_EN);
    i2c_master_write(cmd,data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 200 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t bmp180_write_reg(i2c_port_t i2c_num, uint8_t reg, uint8_t cmd)
{
    uint8_t data_wr[] = {reg, cmd};
    esp_err_t err = bmp180_master_write_slave(i2c_num, data_wr, 2);
    if (err != ESP_OK) {
        ESP_LOGI("[TAG]", "Write [0x%02x] = 0x%02x failed, err = %d", data_wr[0], data_wr[1], err);
    }
    return err;
}


static esp_err_t bmp180_master_read_slave(i2c_port_t i2c_num, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, BMP180_R, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size -1, NACK_VAL); //revoir cette étape
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 200 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t bmp180_read_int16(i2c_port_t i2c_num, uint8_t reg, int16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (int16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] int16 failed, err = %d", reg, err);
    }
    return err;
}

static esp_err_t bmp180_read_uint16(i2c_port_t i2c_num, uint8_t reg, uint16_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[2] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 2);
        if (err == ESP_OK) {
            *value = (uint16_t) ((data_rd[0] << 8) | data_rd[1]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] uint16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t bmp180_read_uint32(i2c_port_t i2c_num, uint8_t reg, uint32_t* value)
{
    esp_err_t err = bmp180_master_write_slave(i2c_num, &reg, 1);
    if (err == ESP_OK) {
        uint8_t data_rd[3] = {0};
        err = bmp180_master_read_slave(i2c_num, data_rd, 3);
        if (err == ESP_OK) {
            *value = (uint32_t) ((data_rd[0] << 16) | (data_rd[1] << 8) | data_rd[2]);
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[ESP]", "Read [0x%02x] uint16 failed, err = %d", reg, err);
    }
    return err;
}


static esp_err_t bmp180_read_uncompensated_temperature(int16_t* temp)
{
    esp_err_t err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_TEMP_CMD);
    vTaskDelay(50/portTICK_PERIOD_MS); //je ne comprends pass la decalration de portTICK_PERIOD_MS je trouve un facteur 10 manquant dans les calculs 
    if (err == ESP_OK) {  
        err = bmp180_read_int16(I2C_NUM_0, BMP180_DATA_TO_READ, temp);
    }
    if (err != ESP_OK) {
        ESP_LOGE("[UTF]", "Read uncompensated temperature failed, err = %d", err);
    }
    return err;
}


static esp_err_t bmp180_calculate_b5(int32_t* b5)
{
    int16_t ut;
    int32_t x1, x2;

    esp_err_t err = bmp180_read_uncompensated_temperature(&ut);
    if (err == ESP_OK) {
        x1 = (ut - (int32_t)ac6) * (int32_t)ac5/pow(2,15); 
        x2 = ((int32_t)mc *pow(2,11)) / (x1 + md);
        *b5 = x1 + x2;
        //ESP_LOGE("[B5]", "Calculate b5 success, b5 = %lld", *b5);
        
        
    } else {
        ESP_LOGE("[B5F]", "Calculate b5 failed, err = %d", err);
    }
    return err;
}

static uint32_t bmp180_read_uncompensated_pressure(uint32_t* up)
{
    esp_err_t err = bmp180_write_reg(I2C_NUM_0, BMP180_CONTROL, BMP180_READ_PRESSURE_CMD + (oversampling << 6));
    vTaskDelay(140/portTICK_PERIOD_MS); //meme raisonnement que précedemment
    if (err == ESP_OK) {
        
        err = bmp180_read_uint32(I2C_NUM_0, BMP180_DATA_TO_READ, up);
        if (err == ESP_OK) {
            *up >>= (8 - oversampling); //voir datasheet pour les formules
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE("[UPF]", "Read uncompensated pressure failed, err = %d", err);
    }
    return err;
}


static void read_tempeature(float *value, int32_t b5)
{
 
    *value= ((b5+8)/pow(2,4))/10.0;

}

static void read_pressure(int32_t *value, int32_t b5)
{
 
    int32_t b3,b6, x1, x2, x3, p;
    uint32_t b4,b7;
    uint32_t up;
    esp_err_t err1;

    
    err1=bmp180_read_uncompensated_pressure(&up);

    if (err1 ==ESP_OK)
    {
        b6 = b5 - 4000;
        x1 = (b2*(pow(b6,2)/pow(2,12)))/(pow(2,11));
        x2 = (ac2*b6)/pow(2,11);
        x3 = x1+x2;
        b3 = ((( (int32_t) ac1*4+x3)<<oversampling)+2)/4 ;
        x1= ((int32_t)ac3*b6)/pow(2,13);
        x2= (b1*((b6*b6)/pow(2,12)))/pow(2,16);
        x3= ((x1+x2)+2)/pow(2,2);
        b4= ac4*(uint32_t)(x3+32768)/pow(2,15);
        b7= ((uint32_t)up-b3)*(50000>>oversampling);

        if (b7<0x80000000)  
        {
            p=(b7*2)/b4;
        }
        else
        {
            p=(b7/b4)*2;
        }

        x1 = (p / pow(2,8)) * (p/pow(2,8));
        x1 = (x1 * 3038)/pow(2,16);
        x2 = (-7357 * p)/pow(2,16);
        p += (x1 + x2 + 3791) /pow(2,4);
        *value = p/100;

    }
    else
     {
        ESP_LOGE("[Error]", "Pressure compensation failed, err = %d", err1);
        }
    //return err1;
    
}

esp_err_t bmp180_init(int pin_sda, int pin_scl)
{
    esp_err_t err;

    i2c_config_t conf;                                    //initialisation de l'i2c au niveau hardware
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    conf.clk_flags = 0;

    err = i2c_param_config(I2C_NUM_0, &conf);              //configuration des paramètres
    if (err != ESP_OK) {
        ESP_LOGE("[INI_ERROR]", "I2C driver configuration failed with error = %d", err);
        
    }
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);  //install le driver i2c
    if (err != ESP_OK) {
        ESP_LOGE("[INI_ERROR]", "I2C driver installation failed with error = %d", err);
       
    }
    ESP_LOGI("[INI_SUCCESS]", "I2C master driver has been installed.");

    uint8_t reg = 0x00;
    err = bmp180_master_write_slave(I2C_NUM_0, &reg, 1); // on test si l'ESP détecte le BMP en envoyant l'adresse 0x00 dedans

    if (err != ESP_OK) {
        ESP_LOGE("[INI_ERROR]", "BMP180 sensor not found at 0x%02x", BMP180_ADDRESS);
        
    }
    else {

    ESP_LOGI("[I2C_SUCCESS]", "BMP180 sensor found at 0x%02x", BMP180_ADDRESS);
    err  = bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC1, &ac1);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC2, &ac2);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_AC3, &ac3);
    err |= bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC4, &ac4);
    err |= bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC5, &ac5);
    err |= bmp180_read_uint16(I2C_NUM_0, BMP180_CAL_AC6, &ac6);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_B1, &b1);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_B2, &b2);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MB, &mb);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MC, &mc);
    err |= bmp180_read_int16(I2C_NUM_0, BMP180_CAL_MD, &md);

    
    ESP_LOGI("[INI_SUCCESS]", "AC1: %d, AC2: %d, AC3: %d, AC4: %d, AC5: %d, AC6: %d", ac1, ac2, ac3, ac4, ac5, ac6);
    ESP_LOGI("[INI_SUCCESS]", "B1: %d, B2: %d, MB: %d, MC: %d, MD: %d", b1, b2, mb, mc, md);
    }
    return ESP_OK;
}



void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : ESP_LOGI("[wakeup]","Wakeup caused by external signal using RTC_IO\r\n"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : ESP_LOGI("[wakeup]","Wakeup caused by external signal using RTC_CNTL\r\n"); break;
    case ESP_SLEEP_WAKEUP_TIMER : ESP_LOGI("[wakeup]","Wakeup caused by timer\r\n"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : ESP_LOGI("[wakeup]","Wakeup caused by touchpad\r\n"); break;
    case ESP_SLEEP_WAKEUP_ULP : ESP_LOGI("[wakeup]","Wakeup caused by ULP program\r\n"); break;
    default : ESP_LOGI("[wakeup]","Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
    
  }
}
