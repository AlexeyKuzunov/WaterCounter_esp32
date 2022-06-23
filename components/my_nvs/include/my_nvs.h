
#ifndef MAIN_MY_NVS_H_
#define MAIN_MY_NVS_H_

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_err.h"

nvs_handle_t my_nvs_init(char * storage);
uint32_t get_counter_nvs(nvs_handle_t my_handle, char * name_count);
void set_counter_nvs(nvs_handle_t my_handle, char * name_count, uint32_t w_c);


#endif /* MAIN_MY_NVS_H_ */
