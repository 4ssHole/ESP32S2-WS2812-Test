#include "esp_log.h"
#include <string.h>

#include "nvs_flash.h"
#include "nvs.h"

#include "NVSHelper.h"

void nvs_init(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
}

void nvs_get(char *key, uint8_t *reciever, int size){
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);

    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        uint8_t buffer = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_u8(my_handle, key, &buffer);
        switch (err) {
            case ESP_OK:
                ESP_LOGE("NVS", "ESPName = %d\n", buffer);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        *reciever = buffer;

        nvs_close(my_handle);
    }
}

void nvs_set(char *key, uint8_t buffer){
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");      

        printf("Updating NVS %s to %i", key, buffer);
        err = nvs_set_u8(my_handle, key, buffer);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        nvs_close(my_handle);
    }
}