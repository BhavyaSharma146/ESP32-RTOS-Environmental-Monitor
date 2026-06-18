#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#define LED_PIN       GPIO_NUM_2

// Define I2C Pins for ESP32
#define I2C_MASTER_SCL_IO           22    // GPIO pin for SCL
#define I2C_MASTER_SDA_IO           21    // GPIO pin for SDA
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          100000  // 100kHz (Standard I2C speed)

#define BMP280_SENSOR_ADDR          0x76  // Standard I2C address for BMP280
#define BMP280_ID_REG_ADDR          0xD0  // ID register address
#define BMP280_RESET_REG_ADDR       0xE0  //Writing 0xB6 to this forces a complete chip reset
#define BMP280_ctrl_REG_ADDR        0xF4    //(Controls power modes and data oversampling)
#define BMP280_pres_REG_ADDR        0xF7    //The beginning of the 6-byte burst for pressure and temperature

uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;

uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;

void i2c_master_init(void);
void bmp280_read_calibration_data(void);
void vSensorTask(void *pvParameters);
void vLoggerTask(void *pvParameters);

typedef struct {
    float temperature;
    float pressure;
} SensorData_t;

QueueHandle_t sensorQueue; //declare the queue handle (globally)

// Variable to share fine temperature structure with pressure math later
int32_t t_fine; 

esp_err_t bmp280_write_register(uint8_t reg_addr, uint8_t value){
    // 1. Create the command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Write the sensor address + Write bit
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);

    // 3. Send the internal register address you want to write to
    i2c_master_write_byte(cmd, reg_addr, true);

    // FIX: You must actually write the data value to the register!
    i2c_master_write_byte(cmd, value, true);

    i2c_master_stop(cmd);

    // 5. Trigger the transmission and wait up to 1 second
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd); 
    
    return ret;
}

esp_err_t bmp280_read_registers(uint8_t start_reg, uint8_t *output_buffer, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Phase 1: Tell the sensor WHICH register room we want to look at
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg, true);

    // Phase 2: Restart and switch to READ mode
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP280_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);

    // Phase 3: Read 'length' number of bytes
    if (length > 1) {
        // Read all bytes up until the second-to-last byte with an ACK
        for (size_t i = 0; i < length - 1; i++) {
            i2c_master_read_byte(cmd, &output_buffer[i], I2C_MASTER_ACK);
        }
    }
    // Read the absolute last byte with a NACK
    i2c_master_read_byte(cmd, &output_buffer[length - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute and clean up
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

float bmp280_compensate_T(int32_t adc_T) {
    int32_t var1, var2;
    float T;
    
    // This looks messy, but it is the exact fixed-point math provided by Bosch
    var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    
    return T / 100.0; // The integer division results in a human-readable float
}

float bmp280_compensate_P(int32_t adc_P) {
    int64_t var1, var2, p;
    
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
    
    // FIX: Cast dig_P1 strictly to (uint64_t) so the compiler cannot sign-extend it to a negative number!
   var1 = ((((((int64_t)1) << 47) + var1)) * ((int64_t)(uint64_t)dig_P1)) >> 33;
    
    if (var1 == 0) {
        return 0; 
    }
    
    p = 1048576LL - adc_P;
    p = (((p << 31) - var2) * 3125LL) / var1; 
    
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;
    
    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    
    return (float)p / 256.0f; 
}

// Function to configure the ESP32's I2C hardware controller
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void bmp280_read_calibration_data(void) {
    uint8_t p_calib_buf[24];
    if (bmp280_read_registers(0x88, p_calib_buf, 24) == ESP_OK) {
        dig_T1 = (uint16_t)((p_calib_buf[1] << 8) | p_calib_buf[0]);
        dig_T2 = (int16_t)((p_calib_buf[3] << 8) | p_calib_buf[2]);
        dig_T3 = (int16_t)((p_calib_buf[5] << 8) | p_calib_buf[4]);

        dig_P1 = (uint16_t)(((uint16_t)p_calib_buf[7]  << 8) | p_calib_buf[6]);
        dig_P2 = (int16_t) (((uint16_t)p_calib_buf[9]  << 8) | p_calib_buf[8]);
        dig_P3 = (int16_t) (((uint16_t)p_calib_buf[11]  << 8) | p_calib_buf[10]);
        dig_P4 = (int16_t) (((uint16_t)p_calib_buf[13]  << 8) | p_calib_buf[12]);
        dig_P5 = (int16_t) (((uint16_t)p_calib_buf[15]  << 8) | p_calib_buf[14]);
        dig_P6 = (int16_t) (((uint16_t)p_calib_buf[17] << 8) | p_calib_buf[16]);
        dig_P7 = (int16_t) (((uint16_t)p_calib_buf[19] << 8) | p_calib_buf[18]);
        dig_P8 = (int16_t) (((uint16_t)p_calib_buf[21] << 8) | p_calib_buf[20]);
        dig_P9 = (int16_t) (((uint16_t)p_calib_buf[23] << 8) | p_calib_buf[22]);
        printf("Calibration data loaded successfully.\n");
    }
    else {
        printf("Failed to read calibration data!\n");
    }

}
void app_main(void) {
    // Initialize Hardware
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize I2C Bus
    i2c_master_init();
    printf("I2C Initialized Successfully.\n");

    bmp280_read_calibration_data();

    // Create a queue capable of containing 5 SensorData_t structures
    sensorQueue = xQueueCreate(5, sizeof(SensorData_t));

    if (sensorQueue == NULL) {
        // Queue wasn't created! Handle error (e.g., print error and loop forever)
        printf("Failed to create queue!\n");
    }

    // Create RTOS Tasks 
    xTaskCreate(vSensorTask, "Sensor Read", 3072, NULL, 2, NULL); 
    xTaskCreate(vLoggerTask, "Logging task", 3072, NULL, 1, NULL); 
}

void vSensorTask(void *pvParameters) {
    SensorData_t data;
    uint8_t raw_data[6]; // Local array for raw bytes

    // Write 0x27 to register 0xF4 to wake up the sensor
    esp_err_t status = bmp280_write_register(0xF4, 0x2F);
    if (status == ESP_OK) {
    printf("Sensor woken up successfully!\n");
    }

    while(1) {
        // Read 6 bytes starting from the pressure register 0xF7
        esp_err_t ret = bmp280_read_registers(0xF7, raw_data, 6);

        if (ret == ESP_OK) {
            // Process the bytes using bit shifts
            int32_t adc_T = ((int32_t)raw_data[3] << 12) | ((int32_t)raw_data[4] << 4) | ((int32_t)raw_data[5] >> 4);
            
            int32_t adc_P = ((int32_t)raw_data[0] << 12) | ((int32_t)raw_data[1] << 4) | ((int32_t)raw_data[2] >> 4);
            
        data.temperature = bmp280_compensate_T(adc_T); 
        data.pressure = bmp280_compensate_P(adc_P) /100.0 ;
        

        // &data means "send a copy of the data at this memory location"
        // 0 means "if the queue is full, don't wait around, just keep moving"

        if (xQueueSend(sensorQueue, &data, 0) == errQUEUE_FULL) {
                printf("Queue full! Data dropped.\n");
            }
    }
     else {
            printf("Failed to read data from sensor.\n");
        }

        // original 2-second delay here
        vTaskDelay(pdMS_TO_TICKS(2000));

    }
}

void vLoggerTask(void *pvParameters) {
    // Create an empty "box" to hold incoming data from the queue
    SensorData_t receivedData;

    while(1) {
        // xQueueReceive sits here and BLOCKS.
        // portMAX_DELAY tells FreeRTOS: "Put this task completely to sleep. 
        // Do not wake it up until an item lands in sensorQueue."
        if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) == pdPASS) {
            
            // The microsecond data arrives, the task instantly wakes up here!
            // Now we read out of our 'receivedData' box:
            printf("--- New Telemetry Received via Queue ---\n");
            printf("Temp: %.2f °C\n", receivedData.temperature);
            printf("Pres: %.2f hPa\n\n", receivedData.pressure);
            
        } // After printing, the loop repeats, hits xQueueReceive, and goes right back to sleep.
    }
}