/* Версия 1.2 — добален буззер (Brauneiger-style tone)
 * Прошивка ESP32 Вариометра
 * Основной функционал:
 * - Считывание данных с барометра BMP3XX и акселерометра (BMA).
 * - Fusion-фильтрация (EMA) данных для быстрого отклика вариометра.
 * - Вывод информации на E-Ink дисплей.
 * - Звуковая/Вибро индикация подъема и спуска.
 * - Управление питанием (Deep Sleep).
 */

#include <esp_sleep.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include "FreeMonoBold36pt7b.h"
#include <cmath>

// --- НАСТРОЙКИ ФИЛЬТРОВ (Fusion) ---
float CFG_TAU_BARO_ALT        = 0.1f;   // Постоянная времени сглаживания высоты (сек)
float CFG_TAU_BARO_VARIO_BASE = 0.3f;   // Базовая постоянная времени вариометра (в спокойном воздухе)
float CFG_TAU_BARO_VARIO_TURB = 0.3f;   // Постоянная времени при турбулентности (динамическая настройка)
float CFG_TAU_ACCEL           = 0.1f;   // Постоянная времени фильтра акселерометра
float CFG_ACCEL_TURB_REF      = 2.0f;   // Эталонное ускорение (м/с²) для определения уровня турбулентности
float CFG_VARIO_SENS          = 1.0f;   // Масштабный коэффициент чувствительности вариометра
float CFG_TAU_COMPLEMENTARY   = 3.0f;   // Постоянная времени комплементарного фильтра (сек)
                                         // > 3c: баро доминирует, < 3c: IMU доминирует
float CFG_TAU_GRAVITY_VEC     = 2.0f;   // Постоянная времени оценки направления гравитации (сек)
                                         // Определяет "вертикаль" независимо от ориентации часов

// --- СИСТЕМНЫЕ НАСТРОЙКИ ---
#define SEALEVELPRESSURE_HPA 1013.25     // Давление на уровне моря для расчета высоты
#define REFRESH_MS 1000                 // Период обновления экрана (мс)
#define LOOP_HZ 50                      // Частота основного цикла обработки (Гц)
#define SINK_TRH -5.0f                  // Порог срабатывания сигнала снижения (м/с)

// --- НАСТРОЙКИ ЭНЕРГОСБЕРЕЖЕНИЯ ---
#define CLOCK_SLEEP_MOTION 60           // Сон с движением (сек)
#define CLOCK_SLEEP_STILL  86400        // Сон без движения (24ч)
#define FLIGHT_DETECT_VZ  1.0f          // Порог Vz для автостарта полёта (м/с)
#define FLIGHT_DETECT_SEC 3             // Сколько секунд Vz > порога для старта
#define LANDING_DETECT_SEC 300          // 5 мин без движения = посадка

// --- НАСТРОЙКИ АКСЕЛЕРОМЕТРА ---
#define GRAVITY_G 9.80665f              // Ускорение свободного падения
#define ACCEL_ALPHA 0.90f               // Коэффициент фильтрации (legacy, не используется в EMA)

// --- ПОРОГИ ЗВУКА И ИМПУЛЬСЫ ---
// Пороги вертикальной скорости (м/с) и соответствующее количество импульсов вибро
const float LIFT_TH[]    = {0.15f, 0.4f, 1.0f, 2.0f, 1000.0f}; 
const int   LIFT_PULSES[] = {1, 2, 3,  0};

// --- НАСТРОЙКИ ВИБРАЦИИ ---
#define V_PULSE 30                      // Длительность импульса вибрации (мс)
#define V_GAP   100                     // Пауза между импульсами в пачке (мс)
#define V_PAUSE 500                     // Длинная пауза между пачками (мс)
#define VIBRO_COOLDOWN_MS 200           // Время "остывания" после вибрации перед замером акселерометра (защита от шума)

// --- ПИНЫ (Hardware) ---
#define BTN_RIGHT 4                     
#define PIN_VARIO_EN 26                 // Пин управления питанием сенсоров
#define BTN_SELECT 35                   
#define BTN_BACK 25                     
#define PIN_VIBRO 13                    
#define PIN_BATT 34                     // АЦП батареи

// --- БУЗЗЕР (Brauneiger-style) ---
// Свободный пин на Watchy V2: GPIO 16, 17, или 32.
// Внимание: на V1/V1.5 GPIO 32 = UP_BTN, используй 16 или 17.
#define BUZZER_PIN     32               // Пин буззера (PWM)
#define BUZZER_CH      1                // LEDC канал

// Настройки тональности (Brauneiger IQ style)
#define BZ_SILENT_MIN  -0.3f            // Нижняя граница тихой зоны (м/с)
#define BZ_SILENT_MAX   0.3f            // Верхняя граница тихой зоны (м/с)
#define BZ_CLIMB_BASE   700              // Базовая частота подъёма (Гц)
#define BZ_CLIMB_MOD    200              // Прибавка частоты на м/с (Гц)
#define BZ_CLIMB_MAX    2200             // Макс частота подъёма (Гц)
#define BZ_SINK_FREQ    500              // Частота снижения (Гц)
#define BZ_SINK_ALARM  -3.0f             // Порог аварийного снижения (м/с)
#define BZ_BEAT_BASE    600              // Базовая длительность такта (мс) при vz=0
#define BZ_BEAT_MIN     80               // Мин длительность такта (мс)
#define BZ_BEAT_RATE    2.0f             // Коэф ускорения бипов от vz

// --- ДИСПЛЕЙ И I2C ---
#define EPD_CS 5                        
#define EPD_RES 9                       
#define EPD_DC 10                       
#define EPD_BUSY 19                     
#define ADDR_BMA 0x18                   
#define ADDR_RTC 0x51
#define ACC_INT_1_PIN 14                // BMA423 INT1 (any-motion wake)
#define RTC_INT_PIN 27                  // PCF8563 INT (alarm wake)

// --- ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ---
enum State { SLEEP, CLOCK, RUNNING, STOPPED, CALIB };
RTC_DATA_ATTR State state = SLEEP;                    // Состояние сохраняется в RTC памяти при сне
RTC_DATA_ATTR unsigned long stopwatchElapsed = 0;     // Накопленное время полета
RTC_DATA_ATTR int lastWakeMinute = -1;                // Минута последнего пробуждения часов
RTC_DATA_ATTR int lastWakeDay = -1;                   // День последнего пробуждения (для 1р/день)

struct SysData {
  float startAlt, alt, vel, maxV, minV, temp;
  float ax, ay, az;
  float gMagRef;                                        // Откалиброванное значение 1G (магнитуда)
  bool track, sensInit, accInit;
  unsigned long tStart, tScreen;
  bool lastSt[3];                                     // Предыдущее состояние кнопок
} data;

int rtc_h, rtc_m, rtc_d, rtc_mon;
Adafruit_BMP3XX bmp;
GxEPD2_BW<GxEPD2_154_D67, 200> display(GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RES, EPD_BUSY));
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t vTaskH = NULL;

// --- КЛАСС ФИЛЬТРАЦИИ ВАРИОМЕТРА (Gravity-aligned IMU + Baro) ---
// Вертикаль определяется по фильтрованному вектору ускорения (gravity vector).
// Проекция текущего ускорения на него даёт альтитудное ускорение независимо
// от ориентации часов. Комплементарный фильтр: IMU(быстрый) + Baro(точный).
// Турбулентность — по магнитуде (все оси).
class VarioEMA {
  float altFilt_        = 0.0f;
  float altPrev_        = 0.0f;
  float varioFilt_      = 0.0f;
  float accelLinFilt_   = 0.0f;
  float velInertial_    = 0.0f;   // Интегрированная скорость от акселерометра
  float velInertialLP_  = 0.0f;   // Low-pass для удаления дрейфа
  float varioBaroLP_    = 0.0f;   // Low-pass baro-варио

  // Gravity vector estimation (LPF on raw accel)
  float gxEst_ = 0.0f, gyEst_ = 0.0f, gzEst_ = 0.0f;  // Низкочастотная оценка гравитации (G)

  bool  inited_       = false;

  float alphaFromTau(float dt, float tau) {
      if (tau <= 0.0f) return 1.0f;
      float a = dt / (tau + dt);
      if (a < 0.0f) a = 0.0f;
      if (a > 1.0f) a = 1.0f;
      return a;
  }

public:
  void init(float initialAlt) {
      altFilt_       = initialAlt;
      altPrev_       = initialAlt;
      varioFilt_     = 0.0f;
      accelLinFilt_  = 0.0f;
      velInertial_   = 0.0f;
      velInertialLP_ = 0.0f;
      varioBaroLP_   = 0.0f;
      gxEst_ = 0.0f; gyEst_ = 0.0f; gzEst_ = 0.0f;
      inited_        = true;
  }

  // Основной метод обновления фильтра.
  // ax_raw, ay_raw, az_raw — сырые показания акселерометра (единицы: G, ±8g диапазон)
  // accelMagMs2            — магнитуда линейного ускорения (м/с²), для турбулентности
  // baroAlt                — сырая высота с барометра
  float update(float ax_raw, float ay_raw, float az_raw,
               float accelMagMs2, float baroAlt, float dt) {
      if (dt < 0.001f) dt = 0.001f;
      if (dt > 0.1f)   dt = 0.1f;

      if (!inited_) {
          init(baroAlt);
      }

      // 1. Оценка направления гравитации (LPF вектора акселерометра)
      //    При любом положении часов этот вектор указывает "вниз"
      float aGrav = alphaFromTau(dt, CFG_TAU_GRAVITY_VEC);
      gxEst_ += aGrav * (ax_raw - gxEst_);
      gyEst_ += aGrav * (ay_raw - gyEst_);
      gzEst_ += aGrav * (az_raw - gzEst_);

      // Нормализация gravity vector
      float gNorm = sqrtf(gxEst_*gxEst_ + gyEst_*gyEst_ + gzEst_*gzEst_);
      if (gNorm > 0.001f) {
          float invG = 1.0f / gNorm;
          gxEst_ *= invG;
          gyEst_ *= invG;
          gzEst_ *= invG;
      } else {
          gxEst_ = 0.0f; gyEst_ = 0.0f; gzEst_ = 1.0f;
      }

      // 2. Проекция текущего ускорения на гравитацию => линейное ускорение по вертикали
      float accelVerticalG = ax_raw*gxEst_ + ay_raw*gyEst_ + az_raw*gzEst_ - 1.0f;
      if (fabsf(accelVerticalG) < 0.02f) accelVerticalG = 0.0f; // deadzone
      float azMs2 = accelVerticalG * GRAVITY_G;

      // 3. Фильтрация магнитуды ускорения (для турбулентности)
      float aAcc = alphaFromTau(dt, CFG_TAU_ACCEL);
      accelLinFilt_ += aAcc * (accelMagMs2 - accelLinFilt_);

      // 4. Фильтрация высоты
      float aAlt = alphaFromTau(dt, CFG_TAU_BARO_ALT);
      altFilt_ += aAlt * (baroAlt - altFilt_);

      // 5. Baro-варио (производная высоты)
      float varioRaw = (altFilt_ - altPrev_) / dt;
      altPrev_ = altFilt_;

      // 6. Адаптация tau варио по турбулентности
      float turb = fabsf(accelLinFilt_);
      float turbNorm = 0.0f;
      if (CFG_ACCEL_TURB_REF > 0.0f) {
          turbNorm = turb / CFG_ACCEL_TURB_REF;
          if (turbNorm > 1.0f) turbNorm = 1.0f;
      }
      float tauVario = CFG_TAU_BARO_VARIO_BASE + turbNorm * CFG_TAU_BARO_VARIO_TURB;
      float aVario = alphaFromTau(dt, tauVario);

      // 7. Low-pass baro-варио (для комплементарного фильтра)
      varioBaroLP_ += aVario * (varioRaw - varioBaroLP_);

      // 8. Комплементарный фильтр: IMU + Baro
      velInertial_ += azMs2 * dt;                    // Интеграция вертикального ускорения

      float aComp = alphaFromTau(dt, CFG_TAU_COMPLEMENTARY);
      velInertialLP_ += aComp * (velInertial_ - velInertialLP_);  // Дрейф

      float velInertialHP = velInertial_ - velInertialLP_;        // HP = без дрейфа

      varioFilt_ = velInertialHP + varioBaroLP_;                  // Fusion

      if (varioFilt_ >  25.0f) varioFilt_ =  25.0f;
      if (varioFilt_ < -25.0f) varioFilt_ = -25.0f;

      return varioFilt_ * CFG_VARIO_SENS;
  }

  float getAltitude() const { return altFilt_; }
  float getVario()    const { return varioFilt_ * CFG_VARIO_SENS; }
  float getAccelLin() const { return accelLinFilt_; }

  // Доступ к оценке гравитации (для отладки)
  void getGravityEst(float &gx, float &gy, float &gz) const {
      gx = gxEst_; gy = gyEst_; gz = gzEst_;
  }
};

VarioEMA varioEMA;

// --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t bcd2dec(uint8_t v) { return ((v/16*10) + (v%16)); }

void readRTC() {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  if(Wire.available()) {
     Wire.read(); // skip seconds
     rtc_m = bcd2dec(Wire.read()&0x7F);
     rtc_h = bcd2dec(Wire.read()&0x3F);
     rtc_d = bcd2dec(Wire.read()&0x3F);
     Wire.read(); // skip
     rtc_mon = bcd2dec(Wire.read()&0x1F);
  }
}

// Установка RTC alarm на +N секунд от текущего времени
void setRTCAlarmSec(int secFromNow) {
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x02); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 6);
  uint8_t s=0, m=0, h=0, d=0, wd=0, mon=0;
  if(Wire.available()) {
     s = Wire.read() & 0x7F;  // секунды
     m = Wire.read() & 0x7F;  // минуты
     h = Wire.read() & 0x3F;  // часы
     d = Wire.read() & 0x3F;  // день
     wd = Wire.read() & 0x07; // день недели
     mon = Wire.read() & 0x1F;// месяц
  }
  // Добавляем секунды
  uint8_t dec_s = (s >> 4) * 10 + (s & 0x0F);
  uint8_t dec_m = (m >> 4) * 10 + (m & 0x0F);
  uint8_t dec_h = (h >> 4) * 10 + (h & 0x0F);
  unsigned long totalSec = dec_h * 3600UL + dec_m * 60UL + dec_s + secFromNow;
  // Обработка переполнения через сутки
  if (totalSec >= 86400UL) totalSec -= 86400UL;
  uint8_t newH = totalSec / 3600;
  uint8_t newM = (totalSec % 3600) / 60;
  uint8_t newS = totalSec % 60;
  // BCD
  uint8_t bcdS = ((newS / 10) << 4) | (newS % 10);
  uint8_t bcdM = ((newM / 10) << 4) | (newM % 10);
  uint8_t bcdH = ((newH / 10) << 4) | (newH % 10);
  // Записываем alarm
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x09); // control/status2
  Wire.write(0x02); // enable alarm, clear AF flag
  Wire.write(0x00); // 0x0A: seconds alarm (0x00 = dont care for seconds)
  Wire.write(bcdM); // 0x0B: minute alarm (0x80 = enable, match minute)
  Wire.write(bcdH); // 0x0C: hour alarm
  Wire.write(0x80); // 0x0D: day alarm (0x80 = disable)
  Wire.write(0x80); // 0x0E: weekday alarm (disable)
  Wire.endTransmission();
  // Включаем alarm interrupt в control/status1
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00); Wire.endTransmission();
  Wire.requestFrom(ADDR_RTC, 1);
  uint8_t cs1 = 0;
  if(Wire.available()) cs1 = Wire.read();
  Wire.beginTransmission(ADDR_RTC); Wire.write(0x00);
  Wire.write(cs1 | 0x02); // set AIE bit
  Wire.endTransmission();
}

// Инициализация BMA423 any-motion interrupt для пробуждения из сна
void initBMAMotionWake() {
  // Устанавливаем any-motion детекцию
  i2cWrite(ADDR_BMA, 0x7C, 0x00); // feature config off
  delay(5);
  // Конфиг any-motion через feature config
  i2cWrite(ADDR_BMA, 0x40, 0x38); // INT1: active low, push-pull
  i2cWrite(ADDR_BMA, 0x41, 0x01); // INT1 map: any-motion → INT1
  // Включаем any-motion detection
  i2cWrite(ADDR_BMA, 0x7C, 0x04);
  delay(10);
}

void initSensors() {
    if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
        bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        data.sensInit = true;
    }
    // Настройка акселерометра BMA (Reset, Config, Enable)
    i2cWrite(ADDR_BMA, 0x7E, 0xB6); delay(20);
    i2cWrite(ADDR_BMA, 0x7C, 0x00); delay(10);
    i2cWrite(ADDR_BMA, 0x40, 0x28);
    i2cWrite(ADDR_BMA, 0x41, 0x01);
    i2cWrite(ADDR_BMA, 0x7D, 0x04); delay(50);
    data.accInit = true;
}

void drawItem(int x, int y, const GFXfont* f, String txt) {
    display.setFont(f);
    if (x < 0) { // Центрирование по горизонтали
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
        x = (display.width() - w) / 2;
    }
    display.setCursor(x, y); display.print(txt);
}

// --- ЗАДАЧА ВАРИОМЕТРА (FreeRTOS Task) ---
// Обрабатывает сенсоры с высокой частотой и управляет вибросигналом
void varioTask(void *p) {
    int pulses = 0;
    bool pulseOn = false;
    unsigned long nextT = 0;
    bool pause = false;

    bool vibroActive = false;
    unsigned long vibroStopT = 0;
    unsigned long lastUpdateMicros = micros();

    float v_smooth = 0.0f;

    for(;;) {
        if (state == RUNNING && data.sensInit) {
            float ax = 0.0f, ay = 0.0f, az = 0.0f;
            float acc_lin_ms2 = 0.0f;

            unsigned long nowMicros = micros();
            float dt = (nowMicros - lastUpdateMicros) / 1000000.0f;
            if (dt < 0.001f) dt = 0.001f;
            if (dt > 0.1f)   dt = 0.1f;
            lastUpdateMicros = nowMicros;

            // Защита от чтения акселерометра во время работы вибромотора (шум)
            bool accSafe = !vibroActive && (millis() - vibroStopT > VIBRO_COOLDOWN_MS);

            if (data.accInit && accSafe) {
                Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                Wire.requestFrom(ADDR_BMA, 6);
                if (Wire.available() >= 6) {
                    int16_t rx = Wire.read() | (Wire.read() << 8);
                    int16_t ry = Wire.read() | (Wire.read() << 8);
                    int16_t rz = Wire.read() | (Wire.read() << 8);
                    ax = rx / 8192.0f;
                    ay = ry / 8192.0f;
                    az = rz / 8192.0f;

                    // Магнитуда — для оценки турбулентности
                    float acc_mag = sqrtf(ax*ax + ay*ay + az*az);
                    float acc_linear_g = acc_mag - data.gMagRef;
                    if (fabsf(acc_linear_g) < 0.02f) acc_linear_g = 0.0f;
                    acc_lin_ms2 = acc_linear_g * GRAVITY_G;
                    // azLinMs2 больше не храним — фильтр сам вычисляет
                    // проекцию на gravity vector из сырых ax, ay, az
                }
            }

            if (bmp.performReading()) {
                float baro_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);

                // Обновление фильтра: сырые ax,ay,az (G), магнитуда (м/с²), baroAlt, dt
                float v = varioEMA.update(ax, ay, az, acc_lin_ms2, baro_alt, dt);

                // Обновление глобальных данных (в критической секции)
                portENTER_CRITICAL(&mux);
                data.alt  = varioEMA.getAltitude();
                data.vel  = v;
                data.temp = bmp.temperature;
                if (accSafe) { data.ax = ax; data.ay = ay; data.az = az; }
                portEXIT_CRITICAL(&mux);

                if (data.track) {
                    if (data.vel > data.maxV) data.maxV = data.vel;
                    if (data.vel < data.minV) data.minV = data.vel;
                }

                v_smooth += (data.vel - v_smooth) * 0.2f; // Дополнительное сглаживание для логики звука

                // --- ЛОГИКА ГЕНЕРАЦИИ ИМПУЛЬСОВ ---
                int reqP = 0;
                float v_check = v_smooth;

                if (v_check <= SINK_TRH) reqP = -1; // Сильное снижение
                else {
                    // Определение количества импульсов по порогам подъема
                    for (int i = 0; i < 4; i++)
                        if (v_check >= LIFT_TH[i] && v_check < LIFT_TH[i+1])
                            reqP = LIFT_PULSES[i];
                }

                unsigned long now = millis();
                bool setVibro = false;

                if (reqP == -1) {
                    // Непрерывный сигнал при снижении
                    setVibro = true; pulses = 0; pulseOn = 1; pause = 0;
                } else if (reqP == 0) {
                    // Тишина
                    setVibro = false; pulses = 0; pulseOn = 0; pause = 0;
                } else {
                    // Генерация пачек импульсов
                    if (pulses == 0 && !pulseOn && !pause) {
                         pulses = reqP; setVibro = true; pulseOn = 1; nextT = now + V_PULSE;
                    }
                    if (now >= nextT) {
                        if (pulseOn) {
                            setVibro = false; pulseOn = 0; pulses--;
                            nextT = now + (pulses > 0 ? V_GAP : V_PAUSE);
                            if(pulses <= 0) pause = true;
                        } else {
                            if (pause) pause = false;
                            else { setVibro = true; pulseOn = 1; nextT = now + V_PULSE; }
                        }
                    } else { setVibro = pulseOn; }
                }

                // Управление физическим пином
                if (setVibro) {
                    digitalWrite(PIN_VIBRO, 1); vibroActive = true;
                } else {
                    digitalWrite(PIN_VIBRO, 0);
                    if (vibroActive) vibroStopT = millis(); // Запомнить момент остановки для защиты акселерометра
                    vibroActive = false;
                }

                // --- БУЗЗЕР: Brauneiger-style тональность ---
                // Частота и каденция пропорциональны Vz
                // Тихая зона: ±0.3 м/с
                // Pulse/pause = 1:1, как у Brauneiger IQ
                static bool bzOn = false;
                static unsigned long bzNextT = 0;
                unsigned long bzNow = millis();

                float bzV = v_smooth; // используем то же сглаживание

                if (bzV >= BZ_SILENT_MAX) {
                    // --- ПОДЪЁМ: возрастающий тон + ускорение бипов ---
                    float climbRate = bzV;
                    if (climbRate > 8.0f) climbRate = 8.0f;

                    // Частота: BZ_CLIMB_BASE + Vz * BZ_CLIMB_MOD (Гц на м/с)
                    int freq = BZ_CLIMB_BASE + (int)(climbRate * BZ_CLIMB_MOD);
                    if (freq > BZ_CLIMB_MAX) freq = BZ_CLIMB_MAX;

                    // Длительность такта (1 бип + 1 пауза)
                    int beatMs = BZ_BEAT_BASE / (1.0f + (climbRate - BZ_SILENT_MAX) * BZ_BEAT_RATE);
                    if (beatMs < BZ_BEAT_MIN) beatMs = BZ_BEAT_MIN;

                    // 1:1 pulse/pause — каждый такт flip
                    if (bzNow >= bzNextT) {
                        bzOn = !bzOn;
                        bzNextT = bzNow + beatMs / 2;
                        if (bzOn) {
                            ledcChangeFrequency(BUZZER_PIN, freq, 8);
                            ledcWrite(BUZZER_PIN, 128);
                        } else {
                            ledcWrite(BUZZER_PIN, 0);
                        }
                    }

                } else if (bzV <= BZ_SILENT_MIN) {
                    // --- СНИЖЕНИЕ: низкий тон, медленные бипы ---
                    float sinkRate = -bzV;
                    if (sinkRate > 5.0f) sinkRate = 5.0f;

                    int freq = BZ_SINK_FREQ;

                    if (sinkRate >= (-BZ_SINK_ALARM)) {
                        // Аварийное снижение — непрерывный тон
                        ledcChangeFrequency(BUZZER_PIN, freq, 8);
                        ledcWrite(BUZZER_PIN, 128);
                        bzOn = true;
                        bzNextT = 0;
                    } else {
                        // Медленные бипы снижения
                        int beatMs = 800 - (int)(sinkRate * 100);
                        if (beatMs < 200) beatMs = 200;

                        if (bzNow >= bzNextT) {
                            bzOn = !bzOn;
                            bzNextT = bzNow + beatMs / 2;
                            if (bzOn) {
                                ledcChangeFrequency(BUZZER_PIN, freq, 8);
                                ledcWrite(BUZZER_PIN, 128);
                            } else {
                                ledcWrite(BUZZER_PIN, 0);
                            }
                        }
                    }

                } else {
                    // --- ТИХАЯ ЗОНА ---
                    if (bzOn) {
                        ledcWrite(BUZZER_PIN, 0);
                        bzOn = false;
                    }
                    bzNextT = 0;
                }
            }
        } else {
            digitalWrite(PIN_VIBRO, 0);
            ledcWrite(BUZZER_PIN, 0); // Выкл буззер в неактивном состоянии
        }

        vTaskDelay(pdMS_TO_TICKS(1000/LOOP_HZ));
    }
}

void drawMain() {
    float dAlt, dVel, dMax, dMin, dTemp;
    portENTER_CRITICAL(&mux);
    dAlt = data.alt; dVel = data.vel; dMax = data.maxV; dMin = data.minV; dTemp = data.temp;
    portEXIT_CRITICAL(&mux);

    char buf[40];
    display.setPartialWindow(0, 0, 200, 200);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);

        // Время полета
        unsigned long t = stopwatchElapsed + (state==RUNNING ? (millis()-data.tStart)/1000 : 0);
        sprintf(buf, "%02lu:%02lu:%02lu", t/3600, (t%3600)/60, t%60);
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%.1fc", dTemp); drawItem(80, 20, &FreeSansBold9pt7b, buf);

        // Заряд батареи
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        sprintf(buf, "%d%%", v>=4.2?100 : (v<=3.3?0 : (int)((v-3.3)*111.1)));
        drawItem(140, 20, &FreeSansBold9pt7b, buf);

        if(state==RUNNING || state==STOPPED) {
            drawItem(25, 50, &FreeSansBold9pt7b, "Start, m   Sea, m");
            sprintf(buf, "%+d", (int)(dAlt - data.startAlt));
            drawItem(30, 90, &FreeSansBold18pt7b, buf);
            sprintf(buf, "%d", (int)dAlt);
            drawItem(130, 90, &FreeSansBold18pt7b, buf);
        }

        drawItem(10, 135, &FreeSansBold9pt7b, "Vario");
        if(data.track) {
            sprintf(buf, "max%+.1f min%+.1f", dMax, dMin);
            drawItem(60, 135, &FreeSansBold9pt7b, buf);
        }
        sprintf(buf, "%+.1f", dVel);
        drawItem(10, 190, &FreeMonoBold36pt7b, buf);
    } while (display.nextPage());
}

void drawClock(bool deep) {
    if(deep) { display.init(115200, true, 2, false); display.setFullWindow(); }
    else display.setPartialWindow(0,0,200,200);

    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        float v = analogReadMilliVolts(PIN_BATT)/1000.0*2.0;
        char buf[20]; sprintf(buf, "%.2fV %d%%", v, (int)((v-3.3)*111.1));
        drawItem(10, 20, &FreeSansBold9pt7b, buf);

        sprintf(buf, "%02d:%02d", rtc_h, rtc_m);
        drawItem(-1, 110, &FreeSansBold24pt7b, buf);
        sprintf(buf, "%02d.%02d", rtc_d, rtc_mon);
        drawItem(-1, 160, &FreeSansBold18pt7b, buf);
    } while (display.nextPage());
}

void goDeepSleep() {
    if(vTaskH) vTaskDelete(vTaskH);
    digitalWrite(PIN_VARIO_EN, 0); digitalWrite(PIN_VIBRO, 0);
    ledcWrite(BUZZER_PIN, 0);
    ledcDetach(BUZZER_PIN);
    display.hibernate(); Wire.end(); WiFi.mode(WIFI_OFF);

    // Пробуждение: кнопка BACK (GPIO25), RTC alarm (GPIO27), BMA motion (GPIO14)
    // Все активны при LOW (кнопка → GND, RTC INT → open-drain, BMA INT → active low)
    const uint64_t wakeMask = BIT64(BTN_BACK) | BIT64(RTC_INT_PIN) | BIT64(ACC_INT_1_PIN);
    esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
}

void setup() {
    pinMode(BTN_SELECT, INPUT);
    pinMode(BTN_RIGHT, INPUT);
    pinMode(BTN_BACK, INPUT);
    pinMode(PIN_VARIO_EN, OUTPUT);
    pinMode(PIN_VIBRO, OUTPUT);
    pinMode(PIN_BATT, INPUT);
    pinMode(ACC_INT_1_PIN, INPUT);  // BMA motion interrupt
    pinMode(RTC_INT_PIN, INPUT);    // RTC alarm interrupt
    digitalWrite(PIN_VARIO_EN, 0);
    digitalWrite(PIN_VIBRO, 0);

    // Инициализация буззера (PWM) — API ESP32 Core 3.x
    ledcAttach(BUZZER_PIN, BZ_CLIMB_BASE, 8);  // канал не нужен, пин сам становится PWM
    ledcWrite(BUZZER_PIN, 0);                   // беззвучно

    Wire.begin();
    display.init(115200, true, 2, false);
    display.setRotation(1);
    setCpuFrequencyMhz(80);
    WiFi.mode(WIFI_OFF);

    readRTC();

    // Определяем причину пробуждения
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    uint64_t wakePin = 0;
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        wakePin = esp_sleep_get_ext1_wakeup_status();
    }

    if (state == RUNNING && cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        // Проснулись во время полёта (например от BMA motion) — продолжаем полёт
        // Это нештатная ситуация, идём в CLOCK
        state = CLOCK;
    }

    if (state == SLEEP || state == CLOCK) {
        if (cause == ESP_SLEEP_WAKEUP_UNDEFINED) {
            // Первый запуск после подачи питания — показываем часы
            drawClock(true);
            state = CLOCK;
            // Идём в loop(), который сразу уложит спать
        }
        else if (cause == ESP_SLEEP_WAKEUP_EXT1) {
            if (wakePin & BIT64(BTN_BACK)) {
                // Пробуждение по кнопке BACK — показываем часы
                drawClock(true);
                state = CLOCK;
            }
            else if (wakePin & BIT64(RTC_INT_PIN)) {
                // Пробуждение по RTC alarm (раз в минуту)
                readRTC();
                if (rtc_m != lastWakeMinute || lastWakeMinute < 0) {
                    lastWakeMinute = rtc_m;
                    drawClock(false);
                }
                // Быстрая проверка: не начался ли полёт?
                digitalWrite(PIN_VARIO_EN, 1);
                delay(10);
                if (bmp.begin_I2C(0x77) || bmp.begin_I2C(0x76)) {
                    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_4X);
                    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
                    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
                    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
                    float altStart = 0;
                    if (bmp.performReading()) altStart = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    float vzSum = 0;
                    int vzCount = 0;
                    for (int i = 0; i < FLIGHT_DETECT_SEC * 10; i++) {
                        if (bmp.performReading()) {
                            float altNow = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                            vzSum += (altNow - altStart);
                            altStart = altNow;
                            vzCount++;
                        }
                        delay(100);
                    }
                    float avgVz = (vzCount > 0) ? vzSum / vzCount : 0;
                    if (avgVz > FLIGHT_DETECT_VZ) {
                        // Обнаружен полёт! Показываем подсказку
                        display.setPartialWindow(0,0,200,200);
                        display.firstPage();
                        do {
                            display.fillScreen(GxEPD_WHITE);
                            display.setTextColor(GxEPD_BLACK);
                            drawItem(-1, 90, &FreeSansBold18pt7b, "FLIGHT!");
                            drawItem(-1, 130, &FreeSansBold9pt7b, "Press SELECT");
                        } while(display.nextPage());
                        digitalWrite(PIN_VARIO_EN, 0);
                        state = CLOCK;
                    } else {
                        digitalWrite(PIN_VARIO_EN, 0);
                    }
                } else {
                    digitalWrite(PIN_VARIO_EN, 0);
                }
                // Если полёт не обнаружен — идём в loop(), который уложит спать
                if (state != RUNNING) state = CLOCK;
            }
            else if (wakePin & BIT64(ACC_INT_1_PIN)) {
                // Пробуждение по движению — показываем часы и ждём
                drawClock(true);
                state = CLOCK;
            }
        }
    } else if (state == STOPPED) {
        drawMain();
    }

    if (state == RUNNING) {
        // Восстановление после глубокого сна в RUNNING — переходим в CLOCK
        state = CLOCK;
        drawClock(true);
    }
}

void loop() {
    // === CLOCK MODE: сон с RTC alarm ===
    if (state == CLOCK) {
        // Определяем: было движение?
        bool hadMotion = false;
        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
        if (cause == ESP_SLEEP_WAKEUP_EXT1) {
            uint64_t pin = esp_sleep_get_ext1_wakeup_status();
            hadMotion = (pin & BIT64(ACC_INT_1_PIN));
        }

        // Если движения не было — спим сутки. Было — минуту.
        int sleepSec = hadMotion ? CLOCK_SLEEP_MOTION : CLOCK_SLEEP_STILL;
        setRTCAlarmSec(sleepSec);
        initBMAMotionWake();
        goDeepSleep();
    }

    // === SLEEP: только по кнопке ===
    if (state == SLEEP) {
        goDeepSleep();
    }

    // === RUNNING/STOPPED: варио режим ===
    if (state == RUNNING && !vTaskH) {
        // Автостарт при обнаружении полёта из setup()
        // или повторный вход в RUNNING
        state = CLOCK; // Сброс — пользователь нажмёт Select
        drawClock(true);
        return;
    }

    bool bSel = digitalRead(BTN_SELECT);
    bool bR   = digitalRead(BTN_RIGHT);
    bool bBack= digitalRead(BTN_BACK);

    if (bBack && !data.lastSt[2]) {
        delay(50);
        if(digitalRead(BTN_BACK)) {
            if (state == RUNNING || state == STOPPED) {
                readRTC(); drawClock(true); state = CLOCK;
                // loop() захватит CLOCK на следующей итерации и уйдёт в сон
                return;
            }
        }
    }

    // Старт полета (Select) — из CLOCK, STOPPED или SLEEP
    if (bSel && !data.lastSt[0] && (state == CLOCK || state == STOPPED || state == SLEEP)) {
        delay(50);
        if(digitalRead(BTN_SELECT)) {
            digitalWrite(PIN_VARIO_EN, 1); delay(50); initSensors();
            display.setPartialWindow(0,0,200,200); display.firstPage();
            do {
                display.fillScreen(GxEPD_WHITE);
                drawItem(20, 90, &FreeSansBold18pt7b, "Calibrating...");
            } while(display.nextPage());

            if(data.sensInit) {
                // --- КАЛИБРОВКА И ПРОГРЕВ ФИЛЬТРА ---
                
                // 1. Инициализируем фильтр начальным значением перед циклом
                if (bmp.performReading()) {
                    varioEMA.init(bmp.readAltitude(SEALEVELPRESSURE_HPA));
                }

                float sumMag = 0.0f;
                const int samples = 100;

                for(int i=0; i<samples; i++) {
                    float _ax = 0.0f, _ay = 0.0f, _az = 0.0f;

                    // Чтение Акселерометра (для калибровки G)
                    Wire.beginTransmission(ADDR_BMA); Wire.write(0x12); Wire.endTransmission();
                    Wire.requestFrom(ADDR_BMA, 6);
                    if(Wire.available()>=6) {
                        int16_t rx = Wire.read()|(Wire.read()<<8);
                        int16_t ry = Wire.read()|(Wire.read()<<8);
                        int16_t rz = Wire.read()|(Wire.read()<<8);
                        _ax = rx/8192.0f;
                        _ay = ry/8192.0f;
                        _az = rz/8192.0f;
                        sumMag += sqrtf(_ax*_ax + _ay*_ay + _az*_az);
                    }

                    // Чтение Барометра и "прогрев" фильтра
                    if(bmp.performReading()) {
                        float rawAlt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                        // Передаём сырые ax,ay,az для инициализации gravity vector
                        // Магнитуда = 0 (стоим на месте, турбулентности нет)
                        varioEMA.update(_ax, _ay, _az, 0.0f, rawAlt, 0.02f);
                    }
                    delay(10);
                }

                // Завершаем калибровку акселерометра
                data.gMagRef = sumMag / samples;
                if(data.gMagRef < 0.5f || data.gMagRef > 1.5f) data.gMagRef = 1.0f;

                // 2. Берем высоту ИЗ ФИЛЬТРА, а не среднее арифметическое
                // Это убирает "ступеньку" при старте
                float finalAlt = varioEMA.getAltitude();

                portENTER_CRITICAL(&mux);
                data.startAlt = finalAlt; 
                data.alt      = finalAlt;
                data.vel      = 0.0f;
                data.maxV     = 0.0f;
                data.minV     = 0.0f;
                portEXIT_CRITICAL(&mux);

                data.track = false;
                stopwatchElapsed = 0;
                data.tStart = millis();
                
                // Запуск задачи обработки (если еще не запущена)
                if(!vTaskH) xTaskCreatePinnedToCore(varioTask, "V", 4096, NULL, 10, &vTaskH, 0);
                state = RUNNING;
            }
        }
    }

    // Сброс времени или стоп полета (Right)
    if (bR && !data.lastSt[1]) {
        delay(50);
        if(digitalRead(BTN_RIGHT)) {
            if (state==RUNNING) {
                stopwatchElapsed += (millis()-data.tStart)/1000;
                state=STOPPED;
                digitalWrite(PIN_VIBRO, 0);
            } else if (state==CLOCK || state==STOPPED) {
                // Сброс RTC в 00:00
                i2cWrite(ADDR_RTC, 0x02, 0);
                i2cWrite(ADDR_RTC, 0x03, 0);
                i2cWrite(ADDR_RTC, 0x04, 0);
                rtc_h=0; rtc_m=0; drawClock(false);
            }
        }
    }

    data.lastSt[0]=bSel; data.lastSt[1]=bR; data.lastSt[2]=bBack;

    // Автоопределение посадки: Vz ≈ 0 и нет движения
    if (state == RUNNING && data.track && (millis() - data.tStart > 10000)) {
        static unsigned long lastActivity = 0;
        float vel = 0;
        portENTER_CRITICAL(&mux);
        vel = data.vel;
        portEXIT_CRITICAL(&mux);
        if (fabsf(vel) < 0.3f && fabsf(data.ax) < 0.1f && fabsf(data.ay) < 0.1f) {
            if (lastActivity == 0) lastActivity = millis();
            if ((millis() - lastActivity) > (LANDING_DETECT_SEC * 1000UL)) {
                // Посадка! Переходим в CLOCK
                stopwatchElapsed += (millis()-data.tStart)/1000;
                if(vTaskH) { vTaskDelete(vTaskH); vTaskH = NULL; }
                digitalWrite(PIN_VARIO_EN, 0);
                digitalWrite(PIN_VIBRO, 0);
                ledcWrite(BUZZER_PIN, 0);
                readRTC(); drawClock(true);
                state = CLOCK;
                return;
            }
        } else {
            lastActivity = 0; // Есть движение — сбрасываем таймер посадки
        }
    }

    if(state == RUNNING && !data.track && (millis() - data.tStart > 5000)) data.track = true;

    // Обновление экрана
    if ((state == RUNNING || state == STOPPED) && (millis() - data.tScreen >= REFRESH_MS)) {
        data.tScreen = millis(); drawMain();
    }
    delay(10);
}
