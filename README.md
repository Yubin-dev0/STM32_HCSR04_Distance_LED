# STM32 HC-SR04 Ultrasonic Sensor with LED Indicator

## English Description
This project demonstrates how to use the **HC-SR04 ultrasonic distance sensor** with an **STM32 NUCLEO-F103RB** board.  
When the measured distance is **less than 10 cm**, the onboard LED (PA5) turns ON; otherwise it remains OFF.  
Distance values are sent over **USART2 (115200 baud)** and can be monitored with PuTTY.

### Pin Map
- **PA9 (D8)** → HC-SR04 **Trig**
- **PA8 (D7)** → HC-SR04 **Echo** (via voltage divider: **1 kΩ (top)**, **2 kΩ (bottom)**)
- **5V** → HC-SR04 **VCC**
- **GND** → HC-SR04 **GND**
- **PA5 (LD2)** → Onboard LED

### Circuit Connection
- Trig is driven directly by PA9 with a 10–15 µs HIGH pulse.  
- Echo is 5V level; use a **1 kΩ / 2 kΩ** divider to bring it to ~3.33 V at **PA8**.  
- Sensor VCC to **5V**, GND to **common GND** (board/sensor/divider bottom all tied).

---

## 한국어 설명
이 프로젝트는 **STM32 NUCLEO-F103RB**와 **HC-SR04 초음파 센서**를 이용해 거리를 측정하는 예제입니다.  
측정 거리가 **10 cm 미만**이면 보드 LED(PA5)가 켜지고, 아니면 꺼집니다.  
거리값은 **USART2 (115200 baud)** 로 전송되며 PuTTY 등 터미널로 확인할 수 있습니다.

### 핀맵 설정
- **PA9 (D8)** → HC-SR04 **Trig**
- **PA8 (D7)** → HC-SR04 **Echo** (**전압 분배기**: **1 kΩ 상단**, **2 kΩ 하단**)
- **5V** → HC-SR04 **VCC**
- **GND** → HC-SR04 **GND**
- **PA5 (LD2)** → 보드 LED

### 회로 연결
- Trig는 PA9에서 **10–15 µs HIGH** 펄스를 직접 출력합니다.  
- Echo는 5V이므로 **1 kΩ / 2 kΩ 분압기**를 통해 약 3.33 V로 낮춰 **PA8**에 입력합니다.  
- 센서의 VCC는 **5V**, GND는 **공통 GND**로 연결합니다.

---

## Demo Video
[![YouTube Demo](https://img.youtube.com/vi/f3xa9UeO0fQ/0.jpg)](https://www.youtube.com/watch?v=f3xa9UeO0fQ)

---

## Core Code
`c
// 1µs timer (TIM2 prescaler set for 1 MHz)
static inline uint32_t micros(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

// Trigger pulse (10–15 µs HIGH)
static void HCSR04_Trigger(void) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 12);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
}

// EXTI callback for PA8 (Echo): rising = start, falling = stop
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_8) {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {
      echo_rise_us = micros();   // rising edge
      echo_done = 0;
    } else {
      echo_fall_us = micros();   // falling edge
      echo_done = 1;
    }
  }
}

// Main loop snippet
__HAL_TIM_SET_COUNTER(&htim2, 0);
HCSR04_Trigger();
uint32_t wait_start = micros();
while (!echo_done && (uint32_t)(micros() - wait_start) < 40000) { /* wait up to 40 ms */ }

float d = -1.0f;
if (echo_done) {
  uint32_t pulse = (uint32_t)(echo_fall_us - echo_rise_us); // µs
  if (pulse > 250 && pulse < 35000) d = (pulse * 0.0343f) * 0.5f; // cm
  else d = -2.0f;
} else d = -1.0f;

// UART print & LED (10 cm threshold)
int n = snprintf(msg, sizeof(msg),
                 (d >= 0.0f) ? "Distance: %.2f cm\r\n" : "Distance: out of range (%.0f)\r\n", d);
HAL_UART_Transmit(&huart2, (uint8_t*)msg, n, 50);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (d > 0 && d < 10.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
