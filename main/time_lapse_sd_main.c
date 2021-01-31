/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_vfs_fat.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

static const char *TAG = "MAIN";

#define TIME_BETWEEN_PIC_MS (1500UL)
#define MOUNT_POINT "/sdcard"

// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN 1
#endif //SPI_DMA_CHAN

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO (22)
#define PIN_NUM_MOSI (19)
#define PIN_NUM_CLK (21)
#define PIN_NUM_CS (0)

#define CAM_PIN_PWDN (-1)  //power down is not used
#define CAM_PIN_RESET (-1) //software reset will be performed
#define CAM_PIN_XCLK (4)
#define CAM_PIN_SIOD (18)
#define CAM_PIN_SIOC (23)

#define CAM_PIN_D7 (36)
#define CAM_PIN_D6 (37)
#define CAM_PIN_D5 (38)
#define CAM_PIN_D4 (39)
#define CAM_PIN_D3 (35)
#define CAM_PIN_D2 (26)
#define CAM_PIN_D1 (13)
#define CAM_PIN_D0 (34)
#define CAM_PIN_VSYNC (5)
#define CAM_PIN_HREF (27)
#define CAM_PIN_PCLK (25)

#define APP_ESP_WIFI_SSID CONFIG_ESP_WIFI_SSID
#define APP_ESP_WIFI_PASS CONFIG_ESP_WIFI_PASSWORD
#define APP_ESP_WIFI_CHANNEL CONFIG_ESP_WIFI_CHANNEL
#define APP_MAX_STA_CONN CONFIG_ESP_MAX_STA_CONN

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_XGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 5, //0-63 lower number means higher quality
    .fb_count = 2      //if more than one, i2s runs in continuous mode. Use only with JPEG
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = APP_ESP_WIFI_SSID,
            .ssid_len = strlen(APP_ESP_WIFI_SSID),
            .channel = APP_ESP_WIFI_CHANNEL,
            .password = APP_ESP_WIFI_PASS,
            .max_connection = APP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK},
    };
    if (strlen(APP_ESP_WIFI_PASS) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             APP_ESP_WIFI_SSID, APP_ESP_WIFI_PASS, APP_ESP_WIFI_CHANNEL);
}

esp_err_t start_file_server(const char *base_path);

void app_main(void)
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024};
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    size_t count = 0;
    FILE *f;
    char file_name[32];
    char dir_name[32];
    uint32_t start_tm, stop_tm, taken;
    int64_t restart_counter = 0;
    int rv;

    //Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(init_camera());

    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
            esp_restart();
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    {
        nvs_handle_t my_handle;
        ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &my_handle));
        DIR *dir = opendir(MOUNT_POINT"/");
        assert(dir);
        struct dirent *entry = readdir(dir);
        if (entry)
        {
            nvs_get_i64(my_handle, "restart_counter", &restart_counter);
            ESP_ERROR_CHECK(nvs_set_i64(my_handle, "restart_counter", restart_counter + 1));
        } else {
            ESP_LOGW(TAG, "SD card emtpy. Resetting restart counter");
            ESP_ERROR_CHECK(nvs_set_i64(my_handle, "restart_counter", restart_counter));
        }
        closedir(dir);
        nvs_close(my_handle);
    }

    wifi_init_softap();

    /* Start the file server */
    ESP_ERROR_CHECK(start_file_server(MOUNT_POINT));

    rv = snprintf(dir_name, sizeof(dir_name), MOUNT_POINT "/%06lld", restart_counter);
    assert(rv > 0);
    assert(rv < sizeof(dir_name));

    rv = mkdir(dir_name, 0);
    assert(rv == 0);

    rv = snprintf(file_name, sizeof(file_name), "%s/%06d.JPG", dir_name, count++);
    assert(rv > 0);
    assert(rv < sizeof(file_name));
    f = fopen(file_name, "wb");
    assert(f);

    while (1)
    {
        start_tm = esp_timer_get_time() / 1000UL;
        ESP_LOGI(TAG, "Taking picture...");
        camera_fb_t *pic = esp_camera_fb_get();

        // use pic->buf to access the image
        ESP_LOGI(TAG, "Picture taken! Its size was: %zu bytes", pic->len);
        ESP_LOGI(TAG, "Saving picture to file: %s", file_name);
        rv = fwrite(pic->buf, sizeof(uint8_t), pic->len, f);
        assert(rv == pic->len);
        esp_camera_fb_return(pic);

        // prepare for next file
        rv = fclose(f);
        assert(rv == 0);
        rv = snprintf(file_name, sizeof(file_name), "%s/%06d.JPG", dir_name, count++);
        assert(rv > 0);
        assert(rv < sizeof(file_name));
        f = fopen(file_name, "wb");
        assert(f);

        stop_tm = esp_timer_get_time() / 1000UL;
        assert(stop_tm > start_tm);
        taken = stop_tm - start_tm;
        ESP_LOGI(TAG, "Picture saved (%ums)", taken);
        if (taken < TIME_BETWEEN_PIC_MS)
        {
            vTaskDelay((TIME_BETWEEN_PIC_MS - taken) / portTICK_RATE_MS);
        }
    }

    // All done, unmount partition and disable SDMMC or SPI peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");

    //deinitialize the bus after all devices are removed
    spi_bus_free(host.slot);
}
