#include "ledController.h"

int red = 0, green = 0, blue = 0;

void initLEDController(){
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(13, RMT_TX_CHANNEL);
    // set counter clock to 40MHz
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    // install ws2812 driver
    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(6, (led_strip_dev_t)config.channel);
    strip = led_strip_new_rmt_ws2812(&strip_config);
    if (!strip) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
}

void setAllLED(uint8_t setRed, uint8_t setGreen, uint8_t setBlue){
    red = setRed;
    green = setGreen;
    blue = setBlue;
    // Clear LED strip (turn off all LEDs)
    ESP_ERROR_CHECK(strip->clear(strip, 100));

    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, red, green, blue));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 1, red, green, blue));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 2, red, green, blue));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 3, red, green, blue));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 4, red, green, blue));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 5, red, green, blue));
    ESP_ERROR_CHECK(strip->refresh(strip, 100));
}

void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

uint8_t getRed(){
    return red;
}

uint8_t getGreen(){
    return green;
}

uint8_t getBlue(){
    return blue;
}