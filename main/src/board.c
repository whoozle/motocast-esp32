#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_log.h"
#include "video.h"

#define I2C_MASTER_NUM (0)
#define I2C_MASTER_TIMEOUT_MS (1000)

static const char *TAG = "board";

esp_err_t waveshare_rgb_lcd_bl_on()
{
    //Configure CH422G to output mode 
    uint8_t write_buf = 0x01;
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        return ret;

    //Pull the backlight pin high to light the screen backlight 
    write_buf = 0x1E;
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/******************************* Turn off the screen backlight **************************************/
esp_err_t waveshare_rgb_lcd_bl_off()
{
    //Configure CH422G to output mode 
    uint8_t write_buf = 0x01;
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
        return ret;

    //Turn off the screen backlight by pulling the backlight pin low 
    write_buf = 0x1A;
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_lcd_panel_handle_t panel_handle = NULL;

void waveshare_init(void) {
    gpio_config_t io_conf = {};

    // Disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // Bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = 1ULL << 4;
    // Set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 8,
        .scl_io_num = 9,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };

    // Configure I2C parameters
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf));

    // Install I2C driver
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0));

    esp_lcd_rgb_panel_config_t panel_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT, // Set the clock source for the panel
        .timings =  {
            .pclk_hz = 16000000, // Pixel clock frequency
            .h_res = 800, // Horizontal resolution
            .v_res = 480, // Vertical resolution
            .hsync_pulse_width = 4, // Horizontal sync pulse width
            .hsync_back_porch = 8, // Horizontal back porch
            .hsync_front_porch = 8, // Horizontal front porch
            .vsync_pulse_width = 4, // Vertical sync pulse width
            .vsync_back_porch = 8, // Vertical back porch
            .vsync_front_porch = 8, // Vertical front porch
            .flags = {
                .pclk_active_neg = 1, // Active low pixel clock
            },
        },
        .data_width = 16, // Data width for RGB
        .bits_per_pixel = 16, // Bits per pixel
        .num_fbs = 2, // Number of frame buffers
        .bounce_buffer_size_px = 16 * 800, // Bounce buffer size in pixels * width
        .sram_trans_align = 4, // SRAM transaction alignment
        .psram_trans_align = 64, // PSRAM transaction alignment
        .hsync_gpio_num = GPIO_NUM_46, // GPIO number for horizontal sync
        .vsync_gpio_num = GPIO_NUM_3, // GPIO number for vertical sync
        .de_gpio_num = GPIO_NUM_5, // GPIO number for data enable
        .pclk_gpio_num = GPIO_NUM_7, // GPIO number for pixel clock
        .disp_gpio_num = -1, // GPIO number for display
        .data_gpio_nums = {
            GPIO_NUM_14,
            GPIO_NUM_38,
            GPIO_NUM_18,
            GPIO_NUM_17,
            GPIO_NUM_10,
            GPIO_NUM_39,
            GPIO_NUM_0,
            GPIO_NUM_45,
            GPIO_NUM_48,
            GPIO_NUM_47,
            GPIO_NUM_21,
            GPIO_NUM_1,
            GPIO_NUM_2,
            GPIO_NUM_42,
            GPIO_NUM_41,
            GPIO_NUM_40,
            },
        .flags = {
            .fb_in_psram = 1, // Use PSRAM for framebuffer
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_LOGI(TAG, "Initialize RGB LCD panel"); // Log the initialization of the RGB LCD panel
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle)); // Initialize the LCD panel
    uint16_t * pixels = malloc(16 * 16 * 2);
    for(uint16_t i = 0; i != 16 * 16; ++i)
        pixels[i] = 0xF100;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_draw_bitmap(panel_handle, 32, 32, 48, 48, pixels));

    waveshare_rgb_lcd_bl_on();
    video_init();
}
