/**
 * Solution to 02 - Blinky Challenge
 * 
 * Toggles LED at different rates using separate tasks.
 * 
 * Date: December 3, 2020
 * Author: Shawn Hymel
 * License: 0BSD
 */

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#include <GyverINA.h>

INA226 ina(0.01f, 8.0f); 

// LED rates
static const int rate_1 = 250;  // LED On ms
static const int rate_2 = 1777;  // LED Off ms
static const int rate_3 = 1500;  // INA Pause ms

// Pins

static const int led_pin_1 = 20;  // Произвольный пин для LED 1
static const int led_pin_RGB = 18;  // RGB Saola

// Our task: blink an LED at one rate
void toggleLED_1(void *parameter) {
  while(1) {
    digitalWrite(led_pin_1, HIGH);
    vTaskDelay(rate_1);
    digitalWrite(led_pin_1, LOW);
    vTaskDelay(rate_1);
  }
}

// Our task 1: blink RGB LED
void toggleLED_RGB(void *parameter) {
  while(1) {
//    digitalWrite(led_pin_RGB, HIGH);
    neopixelWrite(led_pin_RGB,0,0,64); // Green Saola
//    neopixelWrite(led_pin_RGB,64,0,0); // Green 00501
    vTaskDelay(rate_1);
//    digitalWrite(led_pin_RGB, LOW);
    neopixelWrite(led_pin_RGB,0,0,0); // Green
    vTaskDelay(rate_2);
  }
}

// Our task 2: INA + Serial out
void INA_read(void *parameter) {
  while(1) {
     Serial.print(F("Voltage: "));
  Serial.print(ina.getVoltage(), 3);
  Serial.println(F(" V"));

  // Читаем ток
  Serial.print(F("Current: "));
  Serial.print(ina.getCurrent(), 3);
  Serial.println(F(" A"));

  // Читаем мощность
  Serial.print(F("Power: "));
  Serial.print(ina.getPower(), 3);
  Serial.println(F(" W"));

  // Читаем напряжение на шунте
  Serial.print(F("Shunt voltage: "));
  Serial.print(ina.getShuntVoltage(), 6);
  Serial.println(F(" V"));

  Serial.println("");
  vTaskDelay(rate_3);;
  }
}

void setup() {

  // Configure pin
  pinMode(led_pin_1, OUTPUT);
  pinMode(led_pin_RGB, OUTPUT);

  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              INA_read,  // Function to be called
              "INA_read",   // Name of task
              1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              app_cpu);     // Run on one core for demo purposes (ESP32 only)

                // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              toggleLED_RGB,  // Function to be called
              "Toggle RGB",   // Name of task
              1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              app_cpu);     // Run on one core for demo purposes (ESP32 only)

               // Открываем последовательный порт
  Serial.begin(115200);
  Serial.print(F("INA226..."));
                // Проверяем наличие и инициализируем INA226
  if (ina.begin()) {
    Serial.println(F("connected!"));
  } else {
    Serial.println(F("not found!"));
    while (1);
  }
  ina.setSampleTime(INA226_VBUS, INA226_CONV_2116US);   // Повысим время выборки напряжения вдвое
  ina.setSampleTime(INA226_VSHUNT, INA226_CONV_8244US); // Повысим время выборки тока в 8 раз
  ina.setAveraging(INA226_AVG_X4); // Включим встроенное 4х кратное усреднение, по умолчанию усреднения нет 
  Serial.println("");
    
}

void loop() {
  // Do nothing
  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
  
}
