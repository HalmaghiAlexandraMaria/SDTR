/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * Asta e doar un “banner” informativ (comentariu mare).
 * Nu afectează programul.
 *
 * Aici spui: ce placă ai, ce senzori ai, ce pini, ce logică.
 ******************************************************************************
 */
/* USER CODE END Header */

#include "main.h"     // Declarații principale din proiect (HAL + config)
#include "gpio.h"     // Funcții generate de Cube pentru GPIO
#include "i2c.h"      // Funcții generate pentru I2C
#include "usart.h"    // Funcții generate pentru UART

/* CMSIS-RTOS2 */
#include "cmsis_os2.h" // API-ul CMSIS RTOS2 (wrapper peste FreeRTOS)

/* FreeRTOS headers (pt critical section) */
#include "FreeRTOS.h"  // Definiții FreeRTOS
#include "task.h"      // Funcții pentru task-uri (ex: suspend/resume scheduler)

#include <string.h>  // strlen, memcpy etc.
#include <stdio.h>   // snprintf, vsnprintf
#include <stdint.h>  // tipuri uint8_t, int32_t etc.
#include <stdlib.h>  // labs (valoare absolută long)
#include <stdarg.h>  // va_list (pentru printf variadic)

/* extern handles from Cube generated files */
extern I2C_HandleTypeDef hi2c1;    // “mânerul” (handle) pentru I2C1 (creat de Cube)
extern UART_HandleTypeDef huart1;  // handle pentru UART1

/* ===================== TIM2 for microseconds ===================== */
TIM_HandleTypeDef htim2; // handle pentru timerul TIM2 (îl folosim ca “ceas” în microsecunde)

/* ===================== PIN MAP ===================== */
// aici pui “numele” pinilor ca să scrii mai ușor în cod
#define LED_PORT GPIOB
#define LED_PIN  GPIO_PIN_0

#define BUZZ_PORT GPIOB
#define BUZZ_PIN  GPIO_PIN_1

#define US_TRIG_PORT GPIOA
#define US_TRIG_PIN  GPIO_PIN_1

#define US_ECHO_PORT GPIOA
#define US_ECHO_PIN  GPIO_PIN_4

/* thresholds */
// praguri (când consideri “aproape” și “temperatură mare”)
#define DIST_THRESHOLD_CM_x100 (3000) // 30.00 cm -> am înmulțit cu 100 ca să evit float
#define TEMP_ALERT_C_x100      (2500) // 25.00 °C -> tot x100
#define PERIODIC_MS            (5000) // la fiecare 5 secunde trimiț periodic

/* ultrasonic practical limits */
// limite realiste pentru senzor (ignori “nonsense”)
#define US_MIN_VALID_CM_x100 (200)   // sub 2cm e zona moartă
#define US_MAX_VALID_CM_x100 (40000) // peste 400cm nu mai crezi

/* anti-spam + debounce */
#define PRESENCE_COOLDOWN_MS     (1500) // după o detectare, aștepți 1.5s până la alta
#define PRESENCE_CONFIRM_COUNT   (1)    // câte citiri consecutive sub prag

/* ultrasonic timing improvements */
// cât timp ții TRIG sus
#define US_TRIG_PULSE_US         (12)
// cât aștepți să devină ECHO HIGH (dacă nu, timeout)
#define US_ECHO_TIMEOUT_US       (30000)
// cât lași ECHO să stea HIGH (dacă stă prea mult, timeout)
#define US_ECHO_HIGH_TIMEOUT_US  (100000)
// pauză între măsurători
#define US_MEASURE_DELAY_MS      (30)
// de câte ori încerci ca să obții o măsurare bună
#define US_READ_RETRIES          (5)

/* ============== BMP280 =================== */
// adrese I2C posibile pentru BMP280 ( cum e legat pinul SDO)
#define BMP280_ADDR_76 (0x76 << 1) // HAL folosește adresa shiftată cu 1
#define BMP280_ADDR_77 (0x77 << 1)

#define BMP280_REG_ID         0xD0 // registru ID
#define BMP280_REG_RESET      0xE0 // registru reset
#define BMP280_REG_CTRL_MEAS  0xF4 // control măsurare
#define BMP280_REG_CONFIG     0xF5 // config (filtre, standby)
#define BMP280_REG_PRESS_MSB  0xF7 // de aici citești presiune+temp raw
#define BMP280_REG_CALIB00    0x88 // de aici începe calibrarea

static uint16_t bmp_addr = BMP280_ADDR_76; // presupun inițial 0x76

//din calibrarea BMP, necesar ca să convertesc raw -> valori reale
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine = 0; // variabilă internă folosită în formulele BMP

/* ========== System prototypes ============ */
void SystemClock_Config(void); // funcția de setare clock MCU
void Error_Handler(void);      // funcție dacă apare o eroare gravă
static void MX_TIM2_Init(void);// init pentru timerul TIM2

/* ===================== GPIO helpers ===================== */

static inline void LED_On(void)     { HAL_GPIO_WritePin(LED_PORT,   LED_PIN,   GPIO_PIN_SET); }
static inline void LED_Off(void)    { HAL_GPIO_WritePin(LED_PORT,   LED_PIN,   GPIO_PIN_RESET); }
static inline void Buzzer_On(void)  { HAL_GPIO_WritePin(BUZZ_PORT,  BUZZ_PIN,  GPIO_PIN_SET); }
static inline void Buzzer_Off(void) { HAL_GPIO_WritePin(BUZZ_PORT,  BUZZ_PIN,  GPIO_PIN_RESET); }

/* Force init PB0/PB1 as outputs (robust even if CubeMX changes config) */
static void GPIO_UserInit_LED_Buzzer(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE(); // porneșc “curentul” pentru portul B (altfel nu merge)

  GPIO_InitTypeDef GPIO_InitStruct = {0}; // struct de configurare pini (pornește cu 0)
  GPIO_InitStruct.Pin   = LED_PIN | BUZZ_PIN;    // selectezi PB0 și PB1
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;   // output push-pull (poate da 0/1)
  GPIO_InitStruct.Pull  = GPIO_NOPULL;           // fără rezistență internă
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   // viteză mică (destul pentru LED/buzzer)
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);        // aplici setarea pe portul B

  LED_Off();      // pornește “curat”: LED stins
  Buzzer_Off();   // buzzer stins
}

/* Force init TRIG as output + ECHO as input with pull-down */
static void GPIO_UserInit_Ultrasonic(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE(); // porneșc “curentul” pentru port A

  /* Configure TRIG pin as output */
  GPIO_InitTypeDef GPIO_InitStruct = {0};    // struct de init pini
  GPIO_InitStruct.Pin   = US_TRIG_PIN;       // PA1
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP; // TRIG e ieșire
  GPIO_InitStruct.Pull  = GPIO_NOPULL;         // fără pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // low speed
  HAL_GPIO_Init(US_TRIG_PORT, &GPIO_InitStruct); // aplic pe portul TRIG
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET); // TRIG pornește LOW

  /* Configure ECHO pin as input with pull-down */
  GPIO_InitStruct.Pin  = US_ECHO_PIN;      // PA4
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // ECHO e intrare (citim)
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;    // IMPORTANT: dacă nimic nu-l ține, îl trage la 0
  HAL_GPIO_Init(US_ECHO_PORT, &GPIO_InitStruct); // aplic pe portul ECHO
}

/* ================== Hooks (debug) ================== */
// dacă FreeRTOS nu poate aloca memorie (malloc eșuează)
void vApplicationMallocFailedHook(void)
{
  __disable_irq(); // opreșc întreruperile (ca să rămân într-un “freeze” controlat)
  while (1)        // buclă infinită
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // clipește LED ca semn de eroare
    for (volatile uint32_t i = 0; i < 60000; i++) { } // delay “brutal”
  }
}

// dacă un task își “mănâncă” stiva (stack overflow)
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;       // “nu folosesc”, evit warning
  (void)pcTaskName;  // la fel
  __disable_irq();   // oprești întreruperile
  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // alt blink (altă viteză)
    for (volatile uint32_t i = 0; i < 220000; i++) { } // delay mai mare
  }
}

/* ============== Static mutex for UART + I2C ================= */
// mutex = “lacăt”: numai un task poate folosi resursa la un moment dat
static osMutexId_t uartMutex;
static osMutexId_t i2cMutex;

// memorie statică pentru mutex (ca să nu depinzi de heap)
static StaticSemaphore_t uartMutexCb;
static StaticSemaphore_t i2cMutexCb;

// atribute mutex (nume + memorie)
static const osMutexAttr_t uartMutexAttr = {
    .name = "uartMutex",
    .cb_mem = &uartMutexCb,
    .cb_size = sizeof(uartMutexCb)};

static const osMutexAttr_t i2cMutexAttr = {
    .name = "i2cMutex",
    .cb_mem = &i2cMutexCb,
    .cb_size = sizeof(i2cMutexCb)};

/* ================ UART send (thread-safe) ================ */
// trimite text pe Bluetooth, în siguranță (cu mutex)
static void BT_Send(const char *s)
{
	//Asta previne mesaje “amestecate” și stări ciudate.
  if (s == NULL) return;                       // dacă nu ai text, ieși
  osMutexAcquire(uartMutex, osWaitForever);    // ia lacătul (așteaptă cât trebuie)
  HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY); // trimite
  osMutexRelease(uartMutex);                   // dă drumul la lacăt
}

// ca printf, dar trimite pe BT
static void BT_Printf(const char *fmt, ...)
{
  char buf[220];              // buffer unde construim mesajul
  va_list ap;                 // listă de argumente variabile
  va_start(ap, fmt);          // începe să citești argumentele după fmt
  vsnprintf(buf, sizeof(buf), fmt, ap); // construiește textul în buf
  va_end(ap);                 // termină lista
  BT_Send(buf);               // trimite textul
}

/* =========== microsecond delay ===================== */
// delay în microsecunde folosind TIM2
static void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);          // pornesc numărătoarea de la 0
  while (__HAL_TIM_GET_COUNTER(&htim2) < us) // aștepț până ajunge la “us”
  {
    ; // nu fac nimic (busy wait)
  }
}

/* =============== I2C helpers (thread-safe) ===================== */
// citește de pe I2C “registru” -> data[], protejat cu mutex
static HAL_StatusTypeDef I2C_Read(uint16_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	//sa nu apara coliziuni
  HAL_StatusTypeDef st;                          // aici reținem rezultatul
  osMutexAcquire(i2cMutex, osWaitForever);       // ia lacăt I2C
  st = HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, len, 300); // citește
  osMutexRelease(i2cMutex);                      // eliberează lacăt
  return st;                                     // spune dacă a mers
}

// scrie 1 byte într-un registru pe I2C
static HAL_StatusTypeDef I2C_Write(uint16_t dev, uint8_t reg, uint8_t val)
{
  HAL_StatusTypeDef st;
  osMutexAcquire(i2cMutex, osWaitForever); // ia lacăt I2C
  st = HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 300); // scrie
  osMutexRelease(i2cMutex); // eliberează lacăt
  return st;
}

/* ===================== BMP280 ===================== */
// încearcă 0x76 și 0x77, și verifică dacă ID-ul pare valid
static uint8_t BMP280_FindAddress(void)
{
  uint8_t id = 0; // aici citim ID-ul senzorului

  // încercăm adresa 0x76
  if (I2C_Read(BMP280_ADDR_76, BMP280_REG_ID, &id, 1) == HAL_OK && (id == 0x58 || id == 0x60))
  {
    bmp_addr = BMP280_ADDR_76; // am găsit senzorul aici
    return 1;                  // succes
  }

  // încercăm adresa 0x77
  if (I2C_Read(BMP280_ADDR_77, BMP280_REG_ID, &id, 1) == HAL_OK && (id == 0x58 || id == 0x60))
  {
    bmp_addr = BMP280_ADDR_77;
    return 1;
  }

  return 0; // nu am găsit
}

// citește calibrarea (24 bytes) și îi pune în variabilele dig_T*, dig_P*
static void BMP280_ReadCalib(void)
{
  uint8_t b[24]; // buffer 24 bytes
  (void)I2C_Read(bmp_addr, BMP280_REG_CALIB00, b, 24); // citim calibrările

  // construim numere 16-bit din 2 bytes (LSB + MSB)
  dig_T1 = (uint16_t)((b[1] << 8) | b[0]);
  dig_T2 = (int16_t )((b[3] << 8) | b[2]);
  dig_T3 = (int16_t )((b[5] << 8) | b[4]);

  dig_P1 = (uint16_t)((b[7]  << 8) | b[6]);
  dig_P2 = (int16_t )((b[9]  << 8) | b[8]);
  dig_P3 = (int16_t )((b[11] << 8) | b[10]);
  dig_P4 = (int16_t )((b[13] << 8) | b[12]);
  dig_P5 = (int16_t )((b[15] << 8) | b[14]);
  dig_P6 = (int16_t )((b[17] << 8) | b[16]);
  dig_P7 = (int16_t )((b[19] << 8) | b[18]);
  dig_P8 = (int16_t )((b[21] << 8) | b[20]);
  dig_P9 = (int16_t )((b[23] << 8) | b[22]);
}

// inițializează senzorul: reset + setări + calibrare + verificare ID
static uint8_t BMP280_Init_Sensor(void)
{
  (void)I2C_Write(bmp_addr, BMP280_REG_RESET, 0xB6); // reset senzor
  osDelay(50);                                       // așteaptă să-și revină

  (void)I2C_Write(bmp_addr, BMP280_REG_CONFIG, 0xA0);    // standby 1000ms + filter 16
  (void)I2C_Write(bmp_addr, BMP280_REG_CTRL_MEAS, 0x27); // oversampling x1, normal mode

  BMP280_ReadCalib(); // citește calibrările

  uint8_t id = 0;
  if (I2C_Read(bmp_addr, BMP280_REG_ID, &id, 1) != HAL_OK) return 0; // dacă nu citește ID, fail
  return (id == 0x58 || id == 0x60) ? 1 : 0; // succes dacă ID e bun
}

// citește raw presiune și raw temperatură (neconvertite)
static void BMP280_ReadRaw(int32_t *rawT, int32_t *rawP)
{
  uint8_t d[6]; // 6 bytes: pres(3) + temp(3)

  if (I2C_Read(bmp_addr, BMP280_REG_PRESS_MSB, d, 6) != HAL_OK)
  {
    *rawT = 0; // dacă eșuează, pune 0
    *rawP = 0;
    return;
  }

  // BMP pune datele pe 20 de biți (3 bytes, dar ultimii 4 biți sunt “fraction”)
  *rawP = (int32_t)((d[0] << 12) | (d[1] << 4) | (d[2] >> 4));
  *rawT = (int32_t)((d[3] << 12) | (d[4] << 4) | (d[5] >> 4));
}

/* temperature x100 */
// transformă raw temperatura în °C * 100 (fără float)
// formulele sunt din datasheet BMP280
static int32_t BMP280_Compensate_T_x100(int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;              // “cheia” pentru presiune (BMP cere t_fine)
  return (t_fine * 5 + 128) >> 8;    // rezultatul în °C*100
}

/* pressure in Pa-ish (good for display) */
// transformă raw presiune în Pa (aprox), folosind t_fine + dig_P*
static uint32_t BMP280_Compensate_P_Pa(int32_t adc_P)
{
  int64_t var1 = (int64_t)t_fine - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * (int64_t)dig_P1) >> 33;

  if (var1 == 0) return 0; // protecție: evit împărțirea la 0

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

  return (uint32_t)(p / 256); // Pa
}

/* ===================== Ultrasonic: atomic read + improved timing =========== */
// o singură citire ultrasonică (returnează distanța cm*100 sau cod negativ de eroare)
static int32_t Ultrasonic_ReadOnce_x100(void)
{
  vTaskSuspendAll(); // oprește scheduler-ul (ca să nu fii întrerupt în mijloc)

  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET); // TRIG LOW
  delay_us(10); // mică stabilizare

  // pulse TRIG
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_SET); // TRIG HIGH
  delay_us(US_TRIG_PULSE_US);                                 // ține 12us
  HAL_GPIO_WritePin(US_TRIG_PORT, US_TRIG_PIN, GPIO_PIN_RESET); // TRIG LOW

  delay_us(5); // mică pauză după trigger

  // așteaptă ECHO să devină HIGH
  __HAL_TIM_SET_COUNTER(&htim2, 0);         // timer de la 0
  uint32_t timeout_us = US_ECHO_TIMEOUT_US; // 30ms

  while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_RESET)
  {
    uint32_t elapsed = __HAL_TIM_GET_COUNTER(&htim2);
    if (elapsed > timeout_us)
    {
      xTaskResumeAll(); // repornește scheduler-ul înainte să ieși
      return -2;        // ECHO nu s-a ridicat
    }
  }

  // acum ECHO e HIGH -> măsurăm cât stă HIGH
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  timeout_us = US_ECHO_HIGH_TIMEOUT_US; // 100ms max

  while (HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN) == GPIO_PIN_SET)
  {
    uint32_t elapsed = __HAL_TIM_GET_COUNTER(&htim2);
    if (elapsed > timeout_us)
    {
      xTaskResumeAll();
      return -3; // ECHO prea lung
    }
  }

  uint32_t us = __HAL_TIM_GET_COUNTER(&htim2); // durata HIGH în microsecunde
  xTaskResumeAll(); // repornește scheduler-ul

  // convertim timpul în distanță:
  // dist_cm ≈ us * 0.01715  => dist_cm_x100 ≈ us*1000/583
  int32_t dist_x100 = (int32_t)((us * 1000U) / 583U);

  // verificăm dacă pare realist
  if (dist_x100 < US_MIN_VALID_CM_x100 || dist_x100 > US_MAX_VALID_CM_x100)
    return -4;

  return dist_x100; // OK
}

// (ai această funcție, dar în codul final nu prea o mai folosești)
static void sort5(int32_t *a)
{
  for (int i = 0; i < 5; i++)
    for (int j = i + 1; j < 5; j++)
      if (a[j] < a[i])
      {
        int32_t t = a[i];
        a[i] = a[j];
        a[j] = t;
      }
}

// face 5 citiri, păstrează doar valorile valide și returnează mediana
static int32_t Ultrasonic_GetDistanceCm_x100(void)
{
  int32_t v[US_READ_RETRIES]; // aici ținem citirile (5)
  int valid_count = 0;        // câte sunt bune
  int error_echo_timeout = 0; // -2
  int error_echo_high_timeout = 0; // -3
  int error_invalid_range = 0; // -4

  for (int i = 0; i < US_READ_RETRIES; i++)
  {
    v[i] = Ultrasonic_ReadOnce_x100(); // încearcă o citire

    if (v[i] > 0) valid_count++;          // e bună
    else if (v[i] == -2) error_echo_timeout++;
    else if (v[i] == -3) error_echo_high_timeout++;
    else if (v[i] == -4) error_invalid_range++;

    if (i < US_READ_RETRIES - 1) osDelay(US_MEASURE_DELAY_MS); // pauză între încercări
  }

  // dacă n-ai destule citiri bune, întoarce o eroare “cea mai probabilă”
  if (valid_count < 2)
  {
    if (error_echo_timeout > error_echo_high_timeout && error_echo_timeout > error_invalid_range)
      return -2;
    else if (error_echo_high_timeout > error_invalid_range)
      return -3;
    else
      return -4;
  }

  // copiem doar valorile valide într-un vector separat
  int32_t valid_v[US_READ_RETRIES];
  int idx = 0;
  for (int i = 0; i < US_READ_RETRIES; i++)
    if (v[i] > 0) valid_v[idx++] = v[i];

  // sortăm valid_v (bubble sort simplu)
  for (int i = 0; i < idx; i++)
    for (int j = i + 1; j < idx; j++)
      if (valid_v[j] < valid_v[i])
      {
        int32_t t = valid_v[i];
        valid_v[i] = valid_v[j];
        valid_v[j] = t;
      }

  return valid_v[idx / 2]; // mediană = “valoarea din mijloc”
}

/* ===================== Shared state ================== */
// două “flag-uri” globale (spun dacă e prezență sau alertă temperatură)
static volatile uint8_t presence_active = 0;
static volatile uint8_t temp_alert_active = 0;

/* ========= Message Queue for Temp/Pres data ===================== */
// struct = “plicul” pe care TempTask îl pune în cutia poștală (coada)
typedef struct
{
  int32_t  temp_x100;      // temperatură * 100
  uint32_t pres_hpa_x100;  // presiune hPa*100 (la tine e numeric egal cu Pa)
  uint32_t timestamp;      // tick count când a fost citit
} TempPresMessage_t;

// queue = cutia poștală
static osMessageQueueId_t tempPresQueue;
static StaticQueue_t tempPresQueueCb; // memorie statică pentru control block
static uint8_t tempPresQueueBuffer[4 * sizeof(TempPresMessage_t)]; // loc pentru 4 mesaje

static const osMessageQueueAttr_t tempPresQueueAttr = {
    .name   = "tempPresQueue",
    .cb_mem = &tempPresQueueCb,
    .cb_size = sizeof(tempPresQueueCb),
    .mq_mem  = tempPresQueueBuffer,
    .mq_size = sizeof(tempPresQueueBuffer)};

/* ===================== Tasks (STATIC) ===================== */
static void TempTask(void *argument);      // prototip task temperatură
static void DistanceTask(void *argument);  // prototip task distanță

static StaticTask_t tempTcb, distTcb; // control block-uri task (static)
static uint32_t tempStack[256];       // stack task temperatură
static uint32_t distStack[256];       // stack task distanță

static const osThreadAttr_t tempAttr = {
    .name = "TempTask",
    .cb_mem = &tempTcb,
    .cb_size = sizeof(tempTcb),
    .stack_mem = tempStack,
    .stack_size = sizeof(tempStack),
    .priority = osPriorityNormal};

static const osThreadAttr_t distAttr = {
    .name = "DistanceTask",
    .cb_mem = &distTcb,
    .cb_size = sizeof(distTcb),
    .stack_mem = distStack,
    .stack_size = sizeof(distStack),
    .priority = osPriorityAboveNormal}; // distanța are prioritate mai mare

// TASK: citește BMP280, face alertă temperatură, trimite periodic și pune date în queue
static void TempTask(void *argument)
{
  (void)argument; // nu folosim argument

  BT_Send("TempTask START\r\n"); // mesaj de start

  // detectează senzorul BMP280
  if (!BMP280_FindAddress())
  {
    BT_Send("BMP280 NOT FOUND @0x76/0x77\r\n"); // nu e găsit
  }
  else
  {
    uint8_t id = 0;
    (void)I2C_Read(bmp_addr, BMP280_REG_ID, &id, 1); // citește ID
    BT_Printf("BMP found @0x%02X, ID=0x%02X\r\n", (bmp_addr >> 1), id); // afișează

    if (BMP280_Init_Sensor()) BT_Send("BMP init OK\r\n"); // init bun
    else                     BT_Send("BMP init FAIL\r\n"); // init rău
  }

  uint32_t lastPeriodic = osKernelGetTickCount(); // când am trimis ultima oară periodic

  for (;;)
  {
    int32_t rawT = 0, rawP = 0;        // valori raw
    BMP280_ReadRaw(&rawT, &rawP);      // citire raw

    int32_t  t_x100 = BMP280_Compensate_T_x100(rawT); // convert temperatură
    uint32_t p_pa   = BMP280_Compensate_P_Pa(rawP);   // convert presiune

    // tu spui “hPa x100 == Pa” (numeric e adevărat: 1 hPa = 100 Pa)
    uint32_t p_hpa_x100 = p_pa;

    // creezi mesaj pentru queue
    TempPresMessage_t msg = {
        .temp_x100 = t_x100,
        .pres_hpa_x100 = p_hpa_x100,
        .timestamp = osKernelGetTickCount()
    };

    // pui în coadă fără să blochezi
    osMessageQueuePut(tempPresQueue, &msg, 0, 0);

    // ALERTĂ TEMPERATURĂ: dacă depășește pragul
    if (t_x100 > TEMP_ALERT_C_x100)
    {
      if (!temp_alert_active) // dacă alerta nu e deja activă
      {
        temp_alert_active = 1; // o activăm
        LED_On();              // aprindem LED

        // 4 beep-uri
        for (int i = 0; i < 4; i++)
        {
          Buzzer_On();  osDelay(120);
          Buzzer_Off(); osDelay(120);
        }

        // trimitem mesaj o singură dată până revine sub prag
        BT_Printf("ALERTA TEMPERATURA! %ld.%02ld °C | %lu.%02lu hPa\r\n",
                  (long)(t_x100 / 100), (long)labs(t_x100 % 100),
                  (unsigned long)(p_hpa_x100 / 100), (unsigned long)(p_hpa_x100 % 100));
      }
    }
    else
    {
      // dacă a coborât sub prag și alerta era activă, o oprim
      if (temp_alert_active)
      {
        temp_alert_active = 0;
        if (!presence_active) LED_Off(); // stingem LED doar dacă NU e prezență
      }
    }

    // TRIMITERE PERIODICĂ la 5 secunde
    uint32_t now = osKernelGetTickCount();
    if ((now - lastPeriodic) >= PERIODIC_MS)
    {
      lastPeriodic = now;
      BT_Printf("Temperatura: %ld.%02ld °C | Presiune: %lu.%02lu hPa\r\n",
                (long)(t_x100 / 100), (long)labs(t_x100 % 100),
                (unsigned long)(p_hpa_x100 / 100), (unsigned long)(p_hpa_x100 % 100));
    }

    osDelay(200); // TempTask rulează de ~5 ori pe secundă
  }
}

// TASK: citește ultrasonic, confirmă prezența, beep + LED + trimite date + debug erori
static void DistanceTask(void *argument)
{
  (void)argument;

  BT_Send("DistanceTask START\r\n");

  osDelay(100); // timp să se stabilizeze senzorul la pornire

  uint32_t lastPresenceTick = 0; // când am detectat ultima oară prezență (pt cooldown)
  uint8_t confirm = 0;           // contor pentru confirmare
  uint32_t error_count = 0;      // contor erori ultrasonic
  uint32_t last_debug_tick = 0;  // când am trimis ultimul debug

  for (;;)
  {
    int32_t dist_x100 = Ultrasonic_GetDistanceCm_x100(); // citește distanța

    uint32_t now = osKernelGetTickCount(); // timpul “acum”

    // dacă e eroare (negativ), facem debug
    if (dist_x100 < 0)
    {
      error_count++;

      // trimite mesaj la fiecare 10 erori sau la 5 secunde
      if (error_count % 10 == 0 || (now - last_debug_tick) > 5000)
      {
        const char *error_msg = "UNKNOWN";
        if (dist_x100 == -2)      error_msg = "ECHO nu raspunde (nu se ridica la HIGH)";
        else if (dist_x100 == -3) error_msg = "ECHO prea lung (timeout HIGH)";
        else if (dist_x100 == -4) error_msg = "Distanta in afara limitelor";
        else                      error_msg = "Eroare generala";

        BT_Printf("US-026 ERROR: %s (erori: %lu)\r\n", error_msg, (unsigned long)error_count);

        // citești și starea pinilor (diagnostic)
        GPIO_PinState trig_state = HAL_GPIO_ReadPin(US_TRIG_PORT, US_TRIG_PIN);
        GPIO_PinState echo_state = HAL_GPIO_ReadPin(US_ECHO_PORT, US_ECHO_PIN);

        BT_Printf("  TRIG(PA1)=%s, ECHO(PA4)=%s\r\n",
                  trig_state == GPIO_PIN_SET ? "HIGH" : "LOW",
                  echo_state == GPIO_PIN_SET ? "HIGH" : "LOW");

        last_debug_tick = now;
      }
    }
    else
    {
      // dacă avem măsurătoare validă, resetăm contor erori
      error_count = 0;

      // debug la 10 secunde: distanța ok
      if ((now - last_debug_tick) > 10000)
      {
        BT_Printf("US-026 OK: Distanta: %ld.%02ld cm\r\n",
                  (long)(dist_x100 / 100), (long)(labs(dist_x100 % 100)));
        last_debug_tick = now;
      }
    }

    // “aproape” = distanță pozitivă și sub prag (<=30cm)
    uint8_t isClose = (dist_x100 > 0 && dist_x100 <= DIST_THRESHOLD_CM_x100) ? 1 : 0;

    // confirmare: dacă e aproape, creștem confirm; altfel îl resetăm
    if (isClose)
    {
      if (confirm < 255) confirm++;
    }
    else
    {
      confirm = 0;
    }

    // prezență dacă confirm >= numărul cerut
    uint8_t pres = (confirm >= PRESENCE_CONFIRM_COUNT) ? 1 : 0;

    if (pres)
    {
      // cooldown anti-spam
      if ((now - lastPresenceTick) >= PRESENCE_COOLDOWN_MS)
      {
        lastPresenceTick = now;
        presence_active = 1;

        // beep scurt
        Buzzer_On();  osDelay(120);
        Buzzer_Off();

        LED_On(); // LED on pentru prezență

        // luăm ultima temperatură/presiune din queue (citind toate mesajele disponibile)
        TempPresMessage_t msg;
        TempPresMessage_t lastMsg;
        int32_t t_x100 = 0;
        uint32_t p_hpa_x100 = 0;
        uint8_t hasData = 0;

        while (osMessageQueueGet(tempPresQueue, &msg, NULL, 0) == osOK)
        {
          lastMsg = msg; // păstrăm ultimul
          hasData = 1;
        }

        if (hasData)
        {
          t_x100 = lastMsg.temp_x100;
          p_hpa_x100 = lastMsg.pres_hpa_x100;
        }

        // trimitem toate datele
        BT_Printf("DETECTARE PREZENTA! TRANSMIT DATE: Distanta: %ld.%02ld cm | Temperatura: %ld.%02ld °C | Presiune: %lu.%02lu hPa\r\n",
                  (long)(dist_x100 / 100), (long)(labs(dist_x100 % 100)),
                  (long)(t_x100 / 100), (long)labs(t_x100 % 100),
                  (unsigned long)(p_hpa_x100 / 100), (unsigned long)(p_hpa_x100 % 100));
      }
    }
    else
    {
      presence_active = 0;
      if (!temp_alert_active) LED_Off(); // stinge LED doar dacă NU e alertă temperatură
    }

    osDelay(120); // ritmul DistanceTask
  }
}

/* ===================== TIM2 init (1MHz) ===================== */
// setăm TIM2 ca să numere în microsecunde (1MHz)
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE(); // pornești clock la TIM2

  htim2.Instance = TIM2;                 // folosim TIM2
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP; // numără în sus
  htim2.Init.Period = 0xFFFF;            // maxim 65535
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // fără divizare extra

  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq(); // frecvența bus APB1
  uint32_t timclk = pclk1;                 // pe L0, cu prescaler=1, timclk=pclk1

  uint32_t presc = (timclk / 1000000U);    // vrem 1MHz -> prescaler = timclk/1MHz
  if (presc == 0) presc = 1;               // protecție

  htim2.Init.Prescaler = (uint16_t)(presc - 1); // prescaler în registru e “-1”

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) // inițializează timerul
  {
    Error_Handler(); // dacă nu merge, intră în eroare
  }

  HAL_TIM_Base_Start(&htim2); // pornește timerul
}

/* ===================== Clock (MSI 4MHz) ===================== */
// setează clock MCU: MSI 4MHz, fără PLL
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE(); // pornești power interface clock

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI; // folosim MSI
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;                   // MSI ON
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;          // 4 MHz
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;             // fără PLL

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI; // sursa SYSCLK = MSI
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;      // fără divizare
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;       // fără divizare

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

/* ===================== main ===================== */
int main(void)
{
  HAL_Init();            // pornește HAL, configurează SysTick etc.
  SystemClock_Config();  // setează clock-ul

  MX_GPIO_Init();               // init GPIO generat de Cube
  GPIO_UserInit_LED_Buzzer();   // forțează PB0/PB1 output (LED+buzzer)
  GPIO_UserInit_Ultrasonic();   // forțează TRIG out + ECHO input pulldown

  MX_I2C1_Init();         // init I2C1 (Cube)
  MX_USART1_UART_Init();  // init UART1 (Cube)
  MX_TIM2_Init();         // init timer microsecunde

  /* quick HW test */
  LED_On();    HAL_Delay(200);
  LED_Off();   HAL_Delay(200);
  Buzzer_On(); HAL_Delay(200);
  Buzzer_Off();HAL_Delay(200);

  const char *hello = "\r\n=== START: Termometru + Presiometru + Ultrasonic (FreeRTOS) ===\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t *)hello, (uint16_t)strlen(hello), HAL_MAX_DELAY);

  osKernelInitialize(); // pornește “motorul” RTOS (dar încă nu rulează task-uri)

  // creezi mutex-urile și queue-ul
  uartMutex = osMutexNew(&uartMutexAttr);
  i2cMutex  = osMutexNew(&i2cMutexAttr);
  tempPresQueue = osMessageQueueNew(4, sizeof(TempPresMessage_t), &tempPresQueueAttr);

  // creez task-urile
  osThreadNew(TempTask, NULL, &tempAttr);
  osThreadNew(DistanceTask, NULL, &distAttr);

  osKernelStart(); // pornește scheduler-ul (de aici task-urile rulează)

  while (1)
  {
    // aici nu mai ajungi (în mod normal), RTOS preia controlul
  }
}

/* ===================== Error handler ===================== */
void Error_Handler(void)
{
  __disable_irq(); // oprești întreruperile
  while (1)
  {
    HAL_GPIO_TogglePin(LED_PORT, LED_PIN); // blink LED ca să vezi că e eroare
    for (volatile uint32_t i = 0; i < 250000; i++) { } // delay
  }
}
