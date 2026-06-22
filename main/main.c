#include <string.h> 
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "ssd1306.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"

#define LED_PIN       GPIO_NUM_2

// Define I2C Pins for ESP32
#define I2C_MASTER_SCL_IO           22    // GPIO pin for SCL
#define I2C_MASTER_SDA_IO           21    // GPIO pin for SDA
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          400000  // 400kHz

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
void vDisplayTask(void *pvParameters);
void vWifiTask(void *pvParameters);

typedef struct {
    float temperature;
    float pressure;
} SensorData_t;

//QueueHandle_t is a variable type that acts as a reference or "pointer" to a specific Queue
QueueHandle_t sensorQueue; //queue for the logger to terminal
QueueHandle_t displayQueue; // New queue for the OLED
QueueHandle_t wifiQueue; //New queue for wifi

static EventGroupHandle_t s_wifi_event_group; //FreeRTOS Event Group

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Hardcode your credentials here temporarily for testing, or use Kconfig later
#define WIFI_SSID      "ACT102632443964"
#define WIFI_PASS      "33659363"
#define MAXIMUM_RETRY  5

static int s_retry_num = 0;

// Global handles for the new I2C framework
i2c_master_bus_handle_t bus_handle; //i2c_master_bus_handle_t is a specific variable type that holds a reference (or pointer) to an entire initialized I2C bus.
i2c_master_dev_handle_t bmp280_dev_handle;

// Variable to share fine temperature structure with pressure math later
int32_t t_fine; 

esp_err_t bmp280_write_register(uint8_t reg_addr, uint8_t value){
    uint8_t write_buf[2] = {reg_addr, value};
    // The new driver handles start, addresses, payload, and stop safely in one line!
    return i2c_master_transmit(bmp280_dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

esp_err_t bmp280_read_registers(uint8_t start_reg, uint8_t *output_buffer, size_t length) {
   // Writes the register address first, then reads back 'length' bytes directly
    return i2c_master_transmit_receive(bmp280_dev_handle, &start_reg, 1, output_buffer, length, pdMS_TO_TICKS(1000));
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

// Function to configure the modern ESP32 I2C driver engine
void i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    // Allocate the unified software driver bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // Bind the BMP280 onto our unified bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &bmp280_dev_handle));
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            printf("[WIFI] Retrying connection to the AP...\n");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            printf("[WIFI] Maximum retries reached. Failed to connect to the AP completely.\n");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("[WIFI] Connected! Got IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    // Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create the default system event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi driver configuration allocation
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register our event handler for Wi-Fi and IP events
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    // Configure credentials
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("wifi_init_sta finished. Waiting for connection...\n");

    // Block here using our event group until connection happens or fails
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
            //portMAX_DELAY makes sure that this code is woken up only if s_wifi_event_group is changed by callback function. this acts as a wall

    if (bits & WIFI_CONNECTED_BIT) {
        printf("[WIFI] Successfully connected to SSID: %s\n", WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        printf("[WIFI] Failed to connect to SSID: %s\n", WIFI_SSID);
    } else {
        printf("[WIFI] UNEXPECTED EVENT\n");
    }
}

void app_main(void) {
    // Initialize NVS (Required for Wi-Fi configurations)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Hardware
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize I2C Bus
    i2c_master_init();
    printf("I2C Initialized Successfully.\n");

    bmp280_read_calibration_data();

    wifi_init_sta(); // Call Wi-Fi initialization function

    // Create a queue capable of containing 5 SensorData_t structures
    sensorQueue = xQueueCreate(5, sizeof(SensorData_t));
    displayQueue = xQueueCreate(5, sizeof(SensorData_t)); // <--- ADD THIS LINE
    wifiQueue = xQueueCreate(5, sizeof(SensorData_t));
    if (sensorQueue == NULL || displayQueue == NULL) {
        // Queue wasn't created! Handle error (e.g., print error and loop forever)
        printf("Failed to create queues!\n");
    }

    // Create RTOS Tasks 
    xTaskCreate(vSensorTask, "Sensor Read", 3072, NULL, 2, NULL); 
    xTaskCreate(vLoggerTask, "LoggerTask", 3072, NULL, 5, NULL);
    xTaskCreate(vDisplayTask, "DisplayTask", 3072, NULL, 1, NULL); 
    xTaskCreate(vWifiTask, "WifiTask", 4096, NULL, 3, NULL); 
}

void vLoggerTask(void *pvParameters) {
    SensorData_t logData;
    while(1) {//portMAX_DELAY ensures this if statement executes only if new data arrives in the queue
        if (xQueueReceive(sensorQueue, &logData, portMAX_DELAY) == pdTRUE) {
            printf("[LOG] Temp: %.2f C, Pres: %.1f hPa\n", logData.temperature, logData.pressure);
        }
    }
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
                printf("[WARN] Logger Queue full! Data dropped.\n");
            }
        
        if (xQueueSend(displayQueue, &data, 0) == errQUEUE_FULL) {
        printf("[WARN] Display Queue full! Data dropped.\n");
        }
        if (xQueueSend(wifiQueue, &data, 0) == errQUEUE_FULL) {
        printf("[WARN] Wifi-Queue full! Data dropped.\n");
        }       
    }
    else {
            printf("Failed to read data from sensor.\n");
        }     
    
    // original 2-second delay here
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    }


void vDisplayTask(void *pvParameters) {
    SensorData_t receivedDisplayData; //holds unpacked numbers from the queue
    char tempStr[20];
    char presStr[20];

    // Reverting to the official library initialization method which runs on driver_ng!
    ssd1306_config_t cfg = {
        .bus = SSD1306_I2C,
        .width = 128,
        .height = 64,
        .fb = NULL,     
        .fb_len = 0,
        .iface.i2c = {
            .port = I2C_NUM_0,       
            .addr = 0x3C,            
            .rst_gpio = GPIO_NUM_NC  
        }
    };

    ssd1306_handle_t dev = NULL; //reference pointer that you will pass to all future display functions so they know which physical display to talk to
    
    // Now this library function works perfectly because your code is not resource-locking Port 0!
    if (ssd1306_new_i2c(&cfg, &dev) != ESP_OK) {
        printf("[ERROR] Failed to initialize SSD1306 Display!\n");
        vTaskDelete(NULL);
        //Passing NULL tells FreeRTOS to delete the current running task and free up its allocated RAM.
    }

    while(1) {
        if (xQueueReceive(displayQueue, &receivedDisplayData, portMAX_DELAY) == pdTRUE) {
            
            snprintf(tempStr, sizeof(tempStr), "Temp: %.2f C", receivedDisplayData.temperature);
            snprintf(presStr, sizeof(presStr), "Pres: %.1f hPa", receivedDisplayData.pressure);
            
            ssd1306_clear(dev);
            
            ssd1306_draw_text(dev, 20, 4, "ENV MONITOR", true);
            ssd1306_draw_line(dev, 0, 16, 127, 16, true); 
            
            ssd1306_draw_text(dev, 10, 28, tempStr, true);
            ssd1306_draw_text(dev, 10, 44, presStr, true);
            
            ssd1306_display(dev);
            //All the previous draw functions only modified the ESP32's local RAM buffer. This line takes that entire image buffer and pushes it across the physical I2C wires to the physical OLED screen all at once
        }
    }
}

void vWifiTask(void *pvParameters) {
    SensorData_t receivedNetData;

    printf("Wi-Fi Task Started. Waiting for data...\n");

    while(1) {
        // Block until sensor data arrives in the wifiQueue
        if (xQueueReceive(wifiQueue, &receivedNetData, portMAX_DELAY) == pdTRUE) {
            // Placeholder: Prove that the task successfully intercepts the data stream
            printf("[WIFI-PREVIEW] Received data from queue! Temp: %.2f\n", receivedNetData.temperature);
            
            // Future network dispatch code will sit right here!
        }
    }
}