#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <sys/time.h>
#include "my_nvs.h"
//#include "esp_err.h"
//#include "timeutils.h"
//#include "freertos/semphr.h"
#include <esp_event.h>
#include <esp_log.h>
#include "wc_wifi.h"
#include <esp_mqtt.h>

/**
 *
 * GPIO status:
 * GPIO5: output
 * GPIO16:  input, pulled up, interrupt from rising edge.
 * GPIO4:  input, pulled up, interrupt from rising edge.
 *
 */

#define LED_GPIO    GPIO_NUM_5
#define LED_GPIO_PIN_SEL  1ULL<<LED_GPIO
#define HOT_GPIO     GPIO_NUM_16
#define COLD_GPIO    GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<HOT_GPIO) | (1ULL<<COLD_GPIO))
#define ESP_INTR_FLAG_DEFAULT 0

uint32_t hot_count = 0, cold_count = 0;

static xQueueHandle gpio_evt_queue = NULL;

nvs_handle_t my_handle;

//static void IRAM_ATTR gpio_isr_handler(void* arg)
static void gpio_isr_handler(void* arg){
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void* arg){
  uint32_t io_num;

  //create a queue to handle gpio event from isr
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

   int cnt = 0;

   for(;;) {
     /*
      * Каждые Х мсек смотрим количество значений в очереди.
      * Если значения есть, в цикле проверяем номер порта gpio в значении
      *  и устанавливаем соответствующий gpio флаг .
      * Если значение флага gpio больше нуля инкрементируем глобальную переменную
      * счетчика и ее значение в nvs.
      */
     cnt++;
     volatile uint8_t sum_messages = uxQueueMessagesWaiting(gpio_evt_queue);
     uint8_t tmp_hot = 0, tmp_cold = 0;
     if (sum_messages > 0) {
       for(uint8_t i = 0; i < sum_messages; i++){
         xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY);
         if (io_num == HOT_GPIO) tmp_hot++;
         if (io_num == COLD_GPIO) tmp_cold++;
       }
       sum_messages = 0;
       if (tmp_hot > 0){
         hot_count++;
         set_counter_nvs(my_handle, "hot", hot_count);
         printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
       }
       if (tmp_cold > 0) {
         cold_count++;
         set_counter_nvs(my_handle, "cold", cold_count);
         printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
       }
     }
     gpio_set_level(LED_GPIO, cnt % 2);
     vTaskDelay(1000 / portTICK_RATE_MS);
   }
   nvs_close(my_handle);
}

static void connect() {
  // start mqtt
  esp_mqtt_start(CONFIG_MQTT_HOST, CONFIG_MQTT_PORT, "esp-mqtt", CONFIG_MQTT_USER, CONFIG_MQTT_PASS);
}

static void process(void *p) {
  for (;;) {
    // publish every second
    esp_mqtt_publish("/home/test", (uint8_t *)"world", 5, 2, false);
    vTaskDelay(CONFIG_PUB_INTERVAL / portTICK_PERIOD_MS);
  }
}

static void restart(void *_) {
  // initial start
  connect();

  for (;;) {
    // restart periodically
    vTaskDelay(CONFIG_RES_INTERVAL / portTICK_PERIOD_MS);
    esp_mqtt_stop();
    connect();
  }
}


static void status_callback(esp_mqtt_status_t status) {
  switch (status) {
    case ESP_MQTT_STATUS_CONNECTED:
      // subscribe
      esp_mqtt_subscribe("/home/test", 2);

      break;

    case ESP_MQTT_STATUS_DISCONNECTED:
    default:
      break;
  }
}

static void message_callback(const char *topic, uint8_t *payload, size_t len) {
  ESP_LOGI("/home/test", "incoming: %s => %s (%d)", topic, payload, (int)len);
}


void app_main(void)
{
  //zero-initialize the config structure.
  gpio_config_t io_conf = {};

  //disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  //set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  //bit mask of the pins that you want to set,e.g.GPIO5
  io_conf.pin_bit_mask = LED_GPIO_PIN_SEL;
  //disable pull-down mode
  io_conf.pull_down_en = 0;
  //disable pull-up mode
  io_conf.pull_up_en = 0;
  //configure GPIO with the given settings
  gpio_config(&io_conf);

   //interrupt of rising edge
   io_conf.intr_type = GPIO_INTR_POSEDGE;
   //bit mask of the pins, use GPIO16/4 here
   io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
   //set as input mode
   io_conf.mode = GPIO_MODE_INPUT;
   //disable pull-up mode
   io_conf.pull_up_en = 0;
   //enable pull-down mode
   io_conf.pull_down_en = 1;
   gpio_config(&io_conf);

   // initialize mqtt
   esp_mqtt_init(status_callback, message_callback, 256, 2000);

   wifi_init();

   gpio_set_pull_mode(HOT_GPIO, GPIO_PULLDOWN_ENABLE);
   //install gpio isr service
   gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(COLD_GPIO, gpio_isr_handler, (void*) COLD_GPIO);
   //hook isr handler for specific gpio pin
   gpio_isr_handler_add(HOT_GPIO, gpio_isr_handler, (void*) HOT_GPIO);



  my_handle = my_nvs_init("test");
  hot_count = get_counter_nvs(my_handle, "hot");
  cold_count = get_counter_nvs(my_handle, "cold");
  printf("hot_count: %d \n cold_count: %d \n", hot_count, cold_count);

  // create mqtt tasks
  xTaskCreate(process, "process", 2048, NULL, 10, NULL);
  xTaskCreate(restart, "restart", 2048, NULL, 10, NULL);

  //start gpio task
  xTaskCreate(gpio_task, "gpio_task", 2048, NULL, tskIDLE_PRIORITY, NULL);
  printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

  while(1) {
      //printf("cnt: %d\n", cnt++);
  }
}
