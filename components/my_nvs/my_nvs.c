#include <stdio.h>
#include "my_nvs.h"


nvs_handle_t my_nvs_init(char * storage){
  nvs_handle_t my_handle = 0;
  esp_err_t err = 0;
  // Initialize NVS
  err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );
  err = nvs_open(storage, NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    return 0;
  }
  else {
    return my_handle;
    printf("Done\n");
  }
}

uint32_t get_counter_nvs(nvs_handle_t my_handle, char * name_count){
  int32_t value_count = 0;
  esp_err_t err = 0;
  err = nvs_get_i32(my_handle, name_count, &value_count);
      switch (err) {
      case ESP_OK:
        printf("Done\n");
        printf("Restart counter = %d\n", value_count);
        //nvs_close(my_handle);
        return value_count;
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default :
        printf("Error (%s) reading!\n", esp_err_to_name(err));
      }
     return -1;
}

void set_counter_nvs(nvs_handle_t my_handle, char * name_count, uint32_t w_c){
  esp_err_t err = 0;
  err = nvs_set_i32(my_handle, name_count , w_c);
  printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
  printf("Committing updates in NVS ... ");
  err = nvs_commit(my_handle);
  printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
}
