#include <Arduino.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <Meshtastic.h>
#include <Preferences.h>
#include <driver/rtc_io.h>
#include <driver/uart.h>
#include <esp_idf_version.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_rtc_time.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>
#include <inttypes.h>
#include <stddef.h>

constexpr uint8_t SERIAL_RX_PIN = 16;
constexpr uint8_t SERIAL_TX_PIN = 17;
constexpr uint32_t SERIAL_BAUD = 9600;
constexpr uint8_t PIN_CLEANING_REQUEST = 26;
constexpr uint8_t BUILTIN_LED_PIN = 2;

constexpr uint8_t PIN_BUTTON = 0;
constexpr uint8_t PIN_RAIN_GAUGE = 14;
constexpr uint8_t PIN_HORIZONTAL_SERVO = 21;
constexpr uint8_t PIN_VERTICAL_SERVO = 19;
constexpr uint8_t PIN_BRUSH_POSITION = 18;
constexpr uint8_t PIN_BRUSH_ROTATION = 22;

constexpr double MM_PER_PULSE = 0.2;
constexpr unsigned long REPORT_INTERVAL_MS = 5000;
constexpr uint8_t FINISHED_WINDOW_CAPACITY = 16;
constexpr unsigned long MEASUREMENT_WINDOW_MS = 60000;
constexpr uint32_t RAIN_DRY_TIMEOUT_MS = 5 * 60 * 1000;
constexpr unsigned long SENSOR_RETRY_DELAY_MS = 5000;
constexpr unsigned long SENSOR_MESSAGE_SPACING_MS = 1000;
constexpr unsigned long SENSOR_UART_DRAIN_TIMEOUT_MS = 250;
constexpr size_t SENSOR_MESSAGE_SIZE = 24;
constexpr size_t EEPROM_SIZE = 64;
constexpr uint32_t PENDING_MESSAGE_MAGIC = 0x53454E53;
constexpr uint64_t CLEANING_WAKE_MASK = 1ULL << PIN_CLEANING_REQUEST;
constexpr uint8_t MAX_PENDING_MEASUREMENTS = 16;
constexpr uint32_t RAIN_DEBOUNCE_US = 20000;
constexpr uint64_t RTC_US_PER_MS = 1000ULL;
constexpr uint64_t RAIN_LOCK_US = RAIN_DRY_TIMEOUT_MS * RTC_US_PER_MS;
constexpr unsigned long SENSOR_RETRY_WINDOW_MS = 2 * 60 * 1000UL;
constexpr unsigned long SENSOR_SLEEP_RETRY_MS = 5 * 60 * 1000UL;
constexpr uint8_t RAM_PENDING_CAPACITY = 16;

constexpr int VERTICAL_HOME = 0;
constexpr int VERTICAL_TARGET = 180;
constexpr unsigned long VERTICAL_MOVE_MS = 2000;

constexpr int BRUSH_HOME = 0;
constexpr int BRUSH_TARGET = 90;
constexpr unsigned long BRUSH_MOVE_MS = 1000;
constexpr int BRUSH_SPEED_US = 1700;
constexpr int SERVO_STOP_US = 1500;

constexpr int HORIZONTAL_FORWARD_US = 1800;
constexpr int HORIZONTAL_BACKWARD_US = 1200;
constexpr unsigned long CLEANING_TIME_MS = 24000;
constexpr int HORIZONTAL_REPEATS = 2;
constexpr unsigned long STEP_TIME_MS =
    CLEANING_TIME_MS / (2 * HORIZONTAL_REPEATS);
constexpr unsigned long CLEANING_MARGIN_MS = 2000;

constexpr unsigned long BUTTON_DEBOUNCE_MS = 50;
constexpr unsigned long LED_BLINK_DURATION_MS = 5000;
constexpr unsigned long LED_BLINK_INTERVAL_MS = 250;

struct PendingSensorMessage {
  uint32_t magic;
  char text[SENSOR_MESSAGE_SIZE];
};

struct PendingQueue {
  uint8_t count;
  uint8_t eepromMigrated;
  char messages[MAX_PENDING_MEASUREMENTS][SENSOR_MESSAGE_SIZE];
};

Servo horizontalServo;
Servo verticalServo;
Servo brushPositionServo;
Servo brushRotationServo;

enum class CleaningPhase {
  Idle,
  MovingVerticalToTarget,
  ExtendingBrush,
  RotatingForward,
  RotatingBackward,
  WaitingBeforeRetract,
  RetractingBrush,
  ReturningVertical,
};

CleaningPhase cleaningPhase = CleaningPhase::Idle;
unsigned long cleaningStepDeadlineMs = 0;
int horizontalRepeat = 0;

enum class ButtonPress { Ready, Debouncing, Consumed };
ButtonPress buttonPress = ButtonPress::Ready;
int rawButtonState = HIGH;
uint64_t buttonChangedUs = 0;
constexpr uint64_t BUTTON_DEBOUNCE_US = BUTTON_DEBOUNCE_MS * RTC_US_PER_MS;

// Protected by rainInterruptMux. UINT64_MAX closes admission during cleaning.
uint64_t cleaningAllowedFromUs = 0;
uint64_t cameraRequestUs = 0;
RTC_DATA_ATTR bool rainWakeOnRelease = false;
RTC_DATA_ATTR bool cameraWakeOnRelease = false;

RTC_DATA_ATTR uint64_t pulseCount = 0;

RTC_DATA_ATTR uint64_t rainDryDeadlineUs = 0;
portMUX_TYPE rainInterruptMux = portMUX_INITIALIZER_UNLOCKED;
// Access these counters under rainInterruptMux. Only foreground code touches NVS.
uint64_t windowStartedUs = 0;
uint32_t windowPulses = 0;
uint32_t finishedWindows[FINISHED_WINDOW_CAPACITY];
uint8_t finishedWindowCount = 0;
uint32_t unreportedWindowPulses = 0;
uint32_t pulsesSinceDrain = 0;
uint64_t rainDebounceUntilUs = 0;
bool rainContactLow = false;

RTC_DATA_ATTR uint64_t transmissionNextRetryUs = 0;

unsigned long lastReportAt = 0;

Preferences pendingPreferences;
PendingQueue pendingQueue = {};
bool pendingQueueWritable = false;
bool meshtasticReady = false;
uint64_t transmissionWindowEndUs = 0;
uint64_t transmissionNextAttemptUs = 0;

bool persistedRainLockActive = false;

RTC_DATA_ATTR char ramPendingMessages[RAM_PENDING_CAPACITY][SENSOR_MESSAGE_SIZE];
RTC_DATA_ATTR uint8_t ramPendingCount = 0;

bool ledBlinking = false;
unsigned long ledBlinkStartedAt = 0;
unsigned long ledLastToggleAt = 0;

void startCleaning();

bool hasPendingMeasurements() {
  return pendingQueue.count > 0 || ramPendingCount > 0;
}

void setPersistedRainLock(bool active) {
  if (persistedRainLockActive == active) {
    return;
  }
  if (pendingPreferences.putBool("rain_lock", active) != 0) {
    persistedRainLockActive = active;
  } else {
    Serial.println("Failed to persist rain lock state.");
  }
}

bool savePendingQueue(const PendingQueue &updated) {
  // One committed NVS value keeps the payloads and count consistent.
  if (pendingPreferences.putBytes("queue_v1", &updated, sizeof(updated)) !=
      sizeof(updated)) {
    Serial.println("Failed to commit measurement queue to NVS.");
    return false;
  }
  pendingQueue = updated;
  return true;
}

void refreshPendingQueue() {
  if (pendingPreferences.isKey("queue_v1")) {
    PendingQueue stored = {};
    if (pendingPreferences.getBytesLength("queue_v1") != sizeof(stored) ||
        pendingPreferences.getBytes("queue_v1", &stored, sizeof(stored)) !=
            sizeof(stored) ||
        stored.count > MAX_PENDING_MEASUREMENTS || stored.eepromMigrated > 1) {
      Serial.println("NVS queue unreadable; preserving it and using RTC RAM.");
      return;
    }
    for (uint8_t i = 0; i < stored.count; ++i) {
      if (stored.messages[i][0] == '\0' ||
          !memchr(stored.messages[i], '\0', SENSOR_MESSAGE_SIZE)) {
        Serial.println("Invalid NVS queue record; preserving the queue.");
        return;
      }
    }
    pendingQueue = stored;
    pendingQueueWritable = true;
    return;
  }

  // Recover all readable old slots, including orphan records left by an
  // interrupted enqueue or dequeue. A stale transmitted record may replay.
  PendingQueue recovered = {};
  const uint8_t head = pendingPreferences.getUChar("head", 0);
  const uint8_t first = head < MAX_PENDING_MEASUREMENTS ? head : 0;
  for (uint8_t offset = 0; offset < MAX_PENDING_MEASUREMENTS; ++offset) {
    const uint8_t slot = (first + offset) % MAX_PENDING_MEASUREMENTS;
    char key[16];
    snprintf(key, sizeof(key), "item_%u", slot);
    const size_t len = pendingPreferences.getString(
        key, recovered.messages[recovered.count], SENSOR_MESSAGE_SIZE);
    if (len > 1) {
      ++recovered.count;
    } else if (pendingPreferences.isKey(key)) {
      Serial.printf("Legacy NVS slot %u is empty or unreadable; preserving it.\n",
                    slot);
    }
  }
  recovered.eepromMigrated =
      pendingPreferences.getBool("legacy_migrated", false);
  pendingQueue = recovered;
  // Keep the old keys intact. A committed snapshot supersedes them on reboot.
  pendingQueueWritable = true;
}

void migrateLegacyPendingSensorMessage() {
  if (!pendingQueueWritable || pendingQueue.eepromMigrated ||
      pendingQueue.count == MAX_PENDING_MEASUREMENTS) {
    return;
  }
  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("Could not open legacy EEPROM.");
    return;
  }
  PendingSensorMessage legacy = {};
  EEPROM.get(0, legacy);
  EEPROM.end();

  PendingQueue updated = pendingQueue;
  if (legacy.magic == PENDING_MESSAGE_MAGIC && legacy.text[0] != '\0') {
    legacy.text[SENSOR_MESSAGE_SIZE - 1] = '\0';
    strlcpy(updated.messages[updated.count++], legacy.text, SENSOR_MESSAGE_SIZE);
  }
  // Commit the copy and the migration marker together.
  updated.eepromMigrated = 1;
  savePendingQueue(updated);
}

void removeRtcPrefix(uint8_t count) {
  if (count == 0) {
    return;
  }
  ramPendingCount -= count;
  memmove(ramPendingMessages, ramPendingMessages + count,
          ramPendingCount * SENSOR_MESSAGE_SIZE);
  memset(ramPendingMessages + ramPendingCount, 0, count * SENSOR_MESSAGE_SIZE);
}

void validateRtcQueue() {
  const uint8_t stored = ramPendingCount;
  uint8_t valid = 0;
  for (uint8_t i = 0; i < stored && i < RAM_PENDING_CAPACITY; ++i) {
    const char *message = ramPendingMessages[i];
    if (message[0] && memchr(message, '\0', SENSOR_MESSAGE_SIZE)) {
      if (valid != i) {
        memcpy(ramPendingMessages[valid], message, SENSOR_MESSAGE_SIZE);
      }
      ++valid;
    }
  }
  if (valid != stored) {
    Serial.printf("Invalid RTC queue; recovered %u readable reports.\n", valid);
    memset(ramPendingMessages + valid, 0,
           (RAM_PENDING_CAPACITY - valid) * SENSOR_MESSAGE_SIZE);
    ramPendingCount = valid;
  }
}

bool enqueuePendingMeasurement(const char *message) {
  if (pendingQueueWritable && pendingQueue.count < MAX_PENDING_MEASUREMENTS) {
    PendingQueue updated = pendingQueue;
    // RTC holds the newer end of the FIFO. Persist its oldest reports
    // before adding this measurement to the same NVS transaction.
    const uint8_t freeSlots = MAX_PENDING_MEASUREMENTS - updated.count;
    const uint8_t promoted = ramPendingCount < freeSlots ? ramPendingCount : freeSlots;
    memcpy(updated.messages + updated.count, ramPendingMessages,
           promoted * SENSOR_MESSAGE_SIZE);
    updated.count += promoted;
    const bool newestFits = updated.count < MAX_PENDING_MEASUREMENTS;
    if (newestFits) {
      strlcpy(updated.messages[updated.count++], message, SENSOR_MESSAGE_SIZE);
    }
    if (savePendingQueue(updated)) {
      removeRtcPrefix(promoted);
      if (newestFits) {
        return true;
      }
    }
  }
  if (ramPendingCount == RAM_PENDING_CAPACITY) {
    return false;
  }
  strlcpy(ramPendingMessages[ramPendingCount], message, SENSOR_MESSAGE_SIZE);
  ++ramPendingCount;
  Serial.println("Measurement stored in RTC RAM (volatile on power loss).");
  return true;
}

bool removeFirstPendingMeasurement() {
  if (pendingQueue.count > 0) {
    PendingQueue updated = pendingQueue;
    --updated.count;
    memmove(updated.messages, updated.messages + 1,
            updated.count * SENSOR_MESSAGE_SIZE);
    memset(updated.messages[updated.count], 0, SENSOR_MESSAGE_SIZE);
    if (!savePendingQueue(updated)) {
      return false;
    }
    migrateLegacyPendingSensorMessage();
  } else if (ramPendingCount > 0) {
    removeRtcPrefix(1);
  }
  return true;
}

void stopRotation() {
  horizontalServo.writeMicroseconds(SERVO_STOP_US);
  brushRotationServo.writeMicroseconds(SERVO_STOP_US);
}

bool IRAM_ATTR windowExpired(uint64_t nowUs) {
  return windowPulses != 0 &&
         nowUs - windowStartedUs >= MEASUREMENT_WINDOW_MS * RTC_US_PER_MS;
}

void IRAM_ATTR captureRainPulse(uint64_t nowUs) {
  if (nowUs < rainDebounceUntilUs) {
    return;
  }
  rainDebounceUntilUs = nowUs + RAIN_DEBOUNCE_US;
  if (windowExpired(nowUs)) {
    if (finishedWindowCount < FINISHED_WINDOW_CAPACITY) {
      finishedWindows[finishedWindowCount++] = windowPulses;
    } else {
      unreportedWindowPulses += windowPulses;
    }
    windowPulses = 0;
  }
  if (windowPulses == 0) {
    windowStartedUs = nowUs;
  }
  ++windowPulses;
  ++pulsesSinceDrain;
}

// Called before attaching the interrupt or with rainInterruptMux held.
void IRAM_ATTR sampleRainContact() {
  const bool low = (REG_READ(GPIO_IN_REG) & (1UL << PIN_RAIN_GAUGE)) == 0;
  if (low && !rainContactLow) {
    captureRainPulse(esp_timer_get_time());
  }
  rainContactLow = low;
}

void IRAM_ATTR rainGaugeInterrupt() {
  portENTER_CRITICAL_ISR(&rainInterruptMux);
  sampleRainContact();
  portEXIT_CRITICAL_ISR(&rainInterruptMux);
}

// All callers hold rainInterruptMux. Read the ESP32 input register directly
// so the GPIO ISR can apply the same admission check while flash is busy.
bool IRAM_ATTR cleaningAllowedLocked(uint64_t requestUs) {
  return requestUs >= cleaningAllowedFromUs &&
         (rainDebounceUntilUs == 0 ||
          requestUs >= rainDebounceUntilUs + RAIN_LOCK_US - RAIN_DEBOUNCE_US) &&
         (REG_READ(GPIO_IN_REG) & (1UL << PIN_RAIN_GAUGE)) != 0;
}

void IRAM_ATTR cameraInterrupt() {
  portENTER_CRITICAL_ISR(&rainInterruptMux);
  const uint64_t nowUs = esp_timer_get_time();
  if (cleaningAllowedLocked(nowUs)) {
    cameraRequestUs = nowUs;
  }
  portEXIT_CRITICAL_ISR(&rainInterruptMux);
}

void IRAM_ATTR buttonInterrupt() {
  portENTER_CRITICAL_ISR(&rainInterruptMux);
  const int level = (REG_READ(GPIO_IN_REG) & (1UL << PIN_BUTTON)) ? HIGH : LOW;
  if (level != rawButtonState) {
    const uint64_t nowUs = esp_timer_get_time();
    if (level == LOW) {
      if (nowUs - buttonChangedUs >= BUTTON_DEBOUNCE_US) {
        buttonPress = ButtonPress::Ready;
      }
      if (!cleaningAllowedLocked(nowUs)) {
        buttonPress = ButtonPress::Consumed;
      } else if (buttonPress == ButtonPress::Ready) {
        buttonPress = ButtonPress::Debouncing;
      }
    }
    rawButtonState = level;
    buttonChangedUs = nowUs;
  }
  portEXIT_CRITICAL_ISR(&rainInterruptMux);
}

bool inputWorkPending() {
  portENTER_CRITICAL(&rainInterruptMux);
  // Reconcile a changed level even if its GPIO interrupt is still pending.
  sampleRainContact();
  const bool pending = windowPulses || finishedWindowCount || cameraRequestUs ||
                       buttonPress == ButtonPress::Debouncing;
  portEXIT_CRITICAL(&rainInterruptMux);
  return pending;
}

void tryDeepSleep() {
  if (ledBlinking || cleaningPhase != CleaningPhase::Idle || inputWorkPending()) {
    return;
  }
  const uint64_t nowUs = esp_rtc_get_time_us();
  if (hasPendingMeasurements() && transmissionNextRetryUs <= nowUs) {
    return;
  }

  const bool rainLow = digitalRead(PIN_RAIN_GAUGE) == LOW;
  const bool cameraLow = digitalRead(PIN_CLEANING_REQUEST) == LOW;

  const gpio_num_t rainPin = static_cast<gpio_num_t>(PIN_RAIN_GAUGE);
  const gpio_num_t cleaningPin = static_cast<gpio_num_t>(PIN_CLEANING_REQUEST);
  if (rtc_gpio_pullup_en(rainPin) != ESP_OK ||
      rtc_gpio_pulldown_dis(rainPin) != ESP_OK ||
      rtc_gpio_pullup_en(cleaningPin) != ESP_OK ||
      rtc_gpio_pulldown_dis(cleaningPin) != ESP_OK) {
    Serial.println("Failed to configure RTC GPIO pulls.");
    return;
  }

  // A held input has already been observed. Wake on release and rearm it.
  esp_err_t result = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if (result == ESP_OK) {
    result = esp_sleep_enable_ext0_wakeup(rainPin, rainLow ? 1 : 0);
  }
  if (result == ESP_OK) {
    result = esp_sleep_enable_ext1_wakeup(
        CLEANING_WAKE_MASK, cameraLow ? ESP_EXT1_WAKEUP_ANY_HIGH
                                      : ESP_EXT1_WAKEUP_ALL_LOW);
  }
  if (result == ESP_OK && hasPendingMeasurements()) {
    result = esp_sleep_enable_timer_wakeup(transmissionNextRetryUs - nowUs);
  }

  if (result != ESP_OK) {
    Serial.printf("Wake source configuration failed: %d\n", result);
    return;
  }

  if (rainLow != (digitalRead(PIN_RAIN_GAUGE) == LOW) ||
      cameraLow != (digitalRead(PIN_CLEANING_REQUEST) == LOW) ||
      inputWorkPending()) {
    return;
  }
  rainWakeOnRelease = rainLow;
  cameraWakeOnRelease = cameraLow;
  esp_deep_sleep_start();
}

void requestCleaning(uint64_t requestUs) {
  portENTER_CRITICAL(&rainInterruptMux);
  const bool accepted = cleaningAllowedLocked(requestUs);
  if (accepted) {
    cleaningAllowedFromUs = UINT64_MAX;
    cameraRequestUs = 0;
  }
  portEXIT_CRITICAL(&rainInterruptMux);
  if (accepted) {
    startCleaning();
  }
}

void finishCleaning() {
  stopRotation();
  horizontalServo.detach();
  verticalServo.detach();
  brushPositionServo.detach();
  brushRotationServo.detach();
  portENTER_CRITICAL(&rainInterruptMux);
  cleaningAllowedFromUs = esp_timer_get_time();
  portEXIT_CRITICAL(&rainInterruptMux);
  cleaningPhase = CleaningPhase::Idle;
}

void startCleaning() {
  cleaningPhase = CleaningPhase::MovingVerticalToTarget;
  horizontalRepeat = 0;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  horizontalServo.setPeriodHertz(50);
  verticalServo.setPeriodHertz(50);
  brushPositionServo.setPeriodHertz(50);
  brushRotationServo.setPeriodHertz(50);
  horizontalServo.attach(PIN_HORIZONTAL_SERVO, 500, 2400);
  verticalServo.attach(PIN_VERTICAL_SERVO, 500, 2400);
  brushPositionServo.attach(PIN_BRUSH_POSITION, 500, 2400);
  brushRotationServo.attach(PIN_BRUSH_ROTATION, 500, 2400);
  if (!horizontalServo.attached() || !verticalServo.attached() ||
      !brushPositionServo.attached() || !brushRotationServo.attached()) {
    finishCleaning();
    Serial.println("Cleaning aborted: failed to attach all servo outputs.");
    return;
  }
  stopRotation();
  brushPositionServo.write(BRUSH_HOME);
  verticalServo.write(VERTICAL_TARGET);
  cleaningStepDeadlineMs = millis() + VERTICAL_MOVE_MS;
  Serial.println("START CZYSZCZENIA: ruch pionowy");
}

void advanceCleaning(CleaningPhase phase, unsigned long durationMs) {
  cleaningPhase = phase;
  cleaningStepDeadlineMs = millis() + durationMs;
}

void updateCleaning(unsigned long now) {
  if (cleaningPhase == CleaningPhase::Idle ||
      static_cast<int32_t>(now - cleaningStepDeadlineMs) < 0) {
    return;
  }
  switch (cleaningPhase) {
    case CleaningPhase::MovingVerticalToTarget:
      brushPositionServo.write(BRUSH_TARGET);
      advanceCleaning(CleaningPhase::ExtendingBrush, BRUSH_MOVE_MS);
      Serial.println("Wysuwanie szczotki");
      break;
    case CleaningPhase::ExtendingBrush:
      brushRotationServo.writeMicroseconds(BRUSH_SPEED_US);
      horizontalServo.writeMicroseconds(HORIZONTAL_FORWARD_US);
      advanceCleaning(CleaningPhase::RotatingForward, STEP_TIME_MS);
      Serial.println("Start obrotow");
      break;
    case CleaningPhase::RotatingForward:
      horizontalServo.writeMicroseconds(HORIZONTAL_BACKWARD_US);
      advanceCleaning(CleaningPhase::RotatingBackward, STEP_TIME_MS);
      break;
    case CleaningPhase::RotatingBackward:
      if (++horizontalRepeat < HORIZONTAL_REPEATS) {
        horizontalServo.writeMicroseconds(HORIZONTAL_FORWARD_US);
        advanceCleaning(CleaningPhase::RotatingForward, STEP_TIME_MS);
      } else {
        stopRotation();
        advanceCleaning(CleaningPhase::WaitingBeforeRetract, CLEANING_MARGIN_MS);
        Serial.println("Stop obrotow");
      }
      break;
    case CleaningPhase::WaitingBeforeRetract:
      brushPositionServo.write(BRUSH_HOME);
      advanceCleaning(CleaningPhase::RetractingBrush, BRUSH_MOVE_MS);
      Serial.println("Chowanie szczotki");
      break;
    case CleaningPhase::RetractingBrush:
      verticalServo.write(VERTICAL_HOME);
      advanceCleaning(CleaningPhase::ReturningVertical, VERTICAL_MOVE_MS);
      Serial.println("Powrot pionowy");
      break;
    case CleaningPhase::ReturningVertical:
      finishCleaning();
      Serial.println("KONIEC CZYSZCZENIA");
      break;
    case CleaningPhase::Idle:
      break;
  }
}

void handleCleaningInputs() {
  portENTER_CRITICAL(&rainInterruptMux);
  uint64_t requestUs = cameraRequestUs;
  cameraRequestUs = 0;
  if (esp_timer_get_time() - buttonChangedUs >= BUTTON_DEBOUNCE_US) {
    if (rawButtonState == HIGH) {
      buttonPress = ButtonPress::Ready;
    } else if (buttonPress == ButtonPress::Debouncing) {
      buttonPress = ButtonPress::Consumed;
      if (buttonChangedUs > requestUs) {
        requestUs = buttonChangedUs;
      }
    }
  }
  portEXIT_CRITICAL(&rainInterruptMux);
  if (requestUs != 0) {
    requestCleaning(requestUs);
  }
}

void renewRainDryLock(uint64_t eventUs) {
  const uint64_t nowUs = esp_timer_get_time();
  // Treat a future timestamp conservatively as a fresh pulse.
  const uint64_t ageUs = nowUs >= eventUs ? nowUs - eventUs : 0;
  if (ageUs >= RAIN_LOCK_US) {
    rainDryDeadlineUs = 0;
    setPersistedRainLock(false);
    return;
  }
  rainDryDeadlineUs = esp_rtc_get_time_us() + (RAIN_LOCK_US - ageUs);
  setPersistedRainLock(true);
}

void updateRainDryLock() {
  if (rainDryDeadlineUs != 0 && esp_rtc_get_time_us() >= rainDryDeadlineUs) {
    rainDryDeadlineUs = 0;
    setPersistedRainLock(false);
    Serial.println("Rainfall cleaning lockout expired.");
  }
}

void queueRainWindow(uint32_t pulses) {
  char message[SENSOR_MESSAGE_SIZE];
  snprintf(message, sizeof(message), "sensor1;%.1f", pulses * MM_PER_PULSE);
  if (enqueuePendingMeasurement(message)) {
    Serial.printf("Queued %s (%lu pulses).\n", message,
                  static_cast<unsigned long>(pulses));
  } else {
    Serial.printf("Storage full or unavailable; dropping new report %s (%lu pulses).\n",
                  message, static_cast<unsigned long>(pulses));
  }
}

void drainRainGaugeInterrupts() {
  uint32_t reports[FINISHED_WINDOW_CAPACITY + 1];
  portENTER_CRITICAL(&rainInterruptMux);
  uint8_t count = finishedWindowCount;
  memcpy(reports, finishedWindows, count * sizeof(reports[0]));
  finishedWindowCount = 0;
  if (windowExpired(esp_timer_get_time())) {
    reports[count++] = windowPulses;
    windowPulses = 0;
  }
  const uint32_t newPulses = pulsesSinceDrain;
  const uint64_t debounceUntilUs = rainDebounceUntilUs;
  const uint32_t lost = unreportedWindowPulses;
  pulsesSinceDrain = 0;
  unreportedWindowPulses = 0;
  portEXIT_CRITICAL(&rainInterruptMux);

  if (newPulses != 0) {
    pulseCount += newPulses;
    renewRainDryLock(debounceUntilUs - RAIN_DEBOUNCE_US);
  }
  for (uint8_t i = 0; i < count; ++i) {
    queueRainWindow(reports[i]);
  }
  if (lost != 0) {
    Serial.printf("Finished-window buffer full; discarded reports covering %lu pulses.\n",
                  static_cast<unsigned long>(lost));
  }
}

void printRainReport() {
  Serial.println();
  Serial.println("------ RAPORT ------");
  Serial.printf("Impulsy: %" PRIu64 "\n", pulseCount);
  Serial.printf("Opad: %.1f mm\n", pulseCount * MM_PER_PULSE);
  Serial.println("--------------------");
}

void onNodeReport(mt_node_t *, mt_nr_progress_t progress) {
  if (progress == MT_NR_DONE) {
    meshtasticReady = true;
    Serial.println("Meshtastic node responded on UART.");
  }
}

void pollMeshtastic() {
  // In 0.0.7, a second config-complete frame can call the callback after
  // the library cleared it. mt_loop dispatches one frame per call.
  extern void (*node_report_callback)(mt_node_t *, mt_nr_progress_t);
  if (!node_report_callback) {
    node_report_callback = [](mt_node_t *, mt_nr_progress_t) {};
  }
  mt_loop(millis());
}

void requestNodeReport() {
  meshtasticReady = false;
  if (!mt_request_node_report(onNodeReport)) {
    Serial.println("Could not request Meshtastic node status over UART.");
  }
}

void updateTransmission() {
  if (!hasPendingMeasurements()) {
    transmissionWindowEndUs = 0;
    transmissionNextRetryUs = 0;
    return;
  }

  const uint64_t rtcNowUs = esp_rtc_get_time_us();
  if (rtcNowUs < transmissionNextRetryUs) {
    return;
  }
  if (transmissionWindowEndUs == 0) {
    transmissionNextRetryUs = 0;
    transmissionNextAttemptUs = rtcNowUs;
    transmissionWindowEndUs = rtcNowUs + SENSOR_RETRY_WINDOW_MS * RTC_US_PER_MS;
    // Probe again even if rainfall kept the CPU awake throughout backoff.
    requestNodeReport();
    return;
  }
  if (rtcNowUs >= transmissionWindowEndUs) {
    transmissionWindowEndUs = 0;
    transmissionNextRetryUs = rtcNowUs + SENSOR_SLEEP_RETRY_MS * RTC_US_PER_MS;
    Serial.println("Meshtastic retry window expired; backing off for five minutes.");
    return;
  }
  if (!meshtasticReady || rtcNowUs < transmissionNextAttemptUs) {
    return;
  }

  const char *message = pendingQueue.count > 0 ? pendingQueue.messages[0]
                                               : ramPendingMessages[0];
  transmissionNextAttemptUs = rtcNowUs + SENSOR_RETRY_DELAY_MS * RTC_US_PER_MS;
  if (!mt_send_text(message, BROADCAST_ADDR, 0)) {
    Serial.println("Meshtastic UART write failed; keeping the pending report.");
    return;
  }

  // The Meshtastic library uses UART1 through Serial1. Verify that the local
  // transmitter drained before retiring the report.
  if (uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(SENSOR_UART_DRAIN_TIMEOUT_MS)) !=
      ESP_OK) {
    Serial.println("Meshtastic UART did not drain; keeping the pending report.");
    return;
  }
  if (removeFirstPendingMeasurement()) {
    transmissionNextAttemptUs = esp_rtc_get_time_us() +
                                SENSOR_MESSAGE_SPACING_MS * RTC_US_PER_MS;
    if (!hasPendingMeasurements()) {
      transmissionWindowEndUs = 0;
    }
  }
}

void updateLed(unsigned long now) {
  if (!ledBlinking) {
    return;
  }

  if (now - ledBlinkStartedAt >= LED_BLINK_DURATION_MS) {
    digitalWrite(BUILTIN_LED_PIN, LOW);
    ledBlinking = false;
    return;
  }

  if (now - ledLastToggleAt >= LED_BLINK_INTERVAL_MS) {
    digitalWrite(BUILTIN_LED_PIN, !digitalRead(BUILTIN_LED_PIN));
    ledLastToggleAt = now;
  }
}

void onMeshText(uint32_t from, uint32_t to, uint8_t channel, const char *text) {
  (void)from;
  (void)to;
  (void)channel;

  if (text == nullptr) {
    return;
  }

  // Meshtastic 0.0.7 handle_mesh_packet passes exactly decoded.payload.bytes.
  // Recover that live nanopb object on ESP32's flat address space: the public
  // callback omits its size. This adapter is specific to the pinned version.
  using Payload = meshtastic_Data_payload_t;
  const uintptr_t objectAddress = reinterpret_cast<uintptr_t>(text) -
                                  offsetof(Payload, bytes);
  const Payload &payload = *reinterpret_cast<const Payload *>(objectAddress);
  constexpr char command[] = "sensor1;c;";
  static_assert(sizeof(payload.bytes) >= sizeof(command) - 1, "Payload too small");
  if (payload.size != sizeof(command) - 1 ||
      memcmp(payload.bytes, command, sizeof(command) - 1) != 0) {
    Serial.println("Wiadomosc odrzucona.");
    return;
  }

  Serial.println("Remote cleaning request received.");
  digitalWrite(BUILTIN_LED_PIN, LOW);
  ledBlinking = true;
  ledBlinkStartedAt = millis();
  ledLastToggleAt = ledBlinkStartedAt;
  requestCleaning(esp_timer_get_time());
}

void setup() {
  // The cached IDF can report simultaneous causes. Older cores expose one.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 5)
  const uint32_t wakeCauses = esp_sleep_get_wakeup_causes();
#else
  const uint32_t wakeCauses = 1UL << esp_sleep_get_wakeup_cause();
#endif
  const bool rainWake = (wakeCauses & (1UL << ESP_SLEEP_WAKEUP_EXT0)) != 0;
  const bool cleaningWake = (wakeCauses & (1UL << ESP_SLEEP_WAKEUP_EXT1)) != 0;
  const bool resumedFromSleep = esp_reset_reason() == ESP_RST_DEEPSLEEP;
  const bool cameraAssertionWake = cleaningWake && !cameraWakeOnRelease;

  const esp_err_t rainRtcResult =
      rtc_gpio_deinit(static_cast<gpio_num_t>(PIN_RAIN_GAUGE));
  const esp_err_t cleaningRtcResult =
      rtc_gpio_deinit(static_cast<gpio_num_t>(PIN_CLEANING_REQUEST));
  pinMode(PIN_RAIN_GAUGE, INPUT_PULLUP);

  // EXT0 records the armed level even if it changed again before setup.
  // In release mode a timer/cleaning wake with the pin still LOW is the old tip.
  const bool rainAssertionWake = rainWake && !rainWakeOnRelease;
  if (rainAssertionWake) {
    // The wake edge predates this boot's clock. Include elapsed startup time
    // in the window and debounce interval using the earliest representable time.
    captureRainPulse(0);
  }
  // Consume the wake assertion (or the previously held LOW on an unrelated
  // wake). An observed HIGH rearms counting, regardless of debounce expiry.
  // A LOW already present after a cold reset has no known falling edge.
  rainContactLow = !resumedFromSleep || rainAssertionWake ||
                   (rainWakeOnRelease && !rainWake);
  sampleRainContact();
  const bool resetWithWetContact = !resumedFromSleep && rainContactLow;
  attachInterrupt(digitalPinToInterrupt(PIN_RAIN_GAUGE), rainGaugeInterrupt, CHANGE);
  portENTER_CRITICAL(&rainInterruptMux);
  sampleRainContact();
  portEXIT_CRITICAL(&rainInterruptMux);

  pinMode(PIN_CLEANING_REQUEST, INPUT_PULLUP);
  const bool cameraWasHigh = digitalRead(PIN_CLEANING_REQUEST) == HIGH;
  attachInterrupt(digitalPinToInterrupt(PIN_CLEANING_REQUEST), cameraInterrupt, FALLING);
  const uint64_t sampleUs = esp_timer_get_time();
  const uint64_t cleaningSampleRtcUs = esp_rtc_get_time_us();
  portENTER_CRITICAL(&rainInterruptMux);
  const bool newCameraLow = digitalRead(PIN_CLEANING_REQUEST) == LOW &&
      (cameraWasHigh || (resumedFromSleep && (!cameraWakeOnRelease || cleaningWake)));
  if (cameraRequestUs == 0 && (cameraAssertionWake || newCameraLow)) {
    const uint64_t edgeUs = cameraAssertionWake ? 1 : sampleUs;
    if (cleaningAllowedLocked(edgeUs)) {
      cameraRequestUs = edgeUs;
    }
  }
  portEXIT_CRITICAL(&rainInterruptMux);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(BUILTIN_LED_PIN, OUTPUT);
  digitalWrite(BUILTIN_LED_PIN, LOW);
  rawButtonState = digitalRead(PIN_BUTTON);
  buttonPress = rawButtonState == LOW ? ButtonPress::Consumed : ButtonPress::Ready;
  buttonChangedUs = esp_timer_get_time();
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), buttonInterrupt, CHANGE);
  lastReportAt = millis();

  Serial.begin(115200);
  if (rainRtcResult != ESP_OK || cleaningRtcResult != ESP_OK) {
    Serial.println("Failed to return a wake pin to digital GPIO mode.");
  }
  validateRtcQueue();
  const bool nvsOpen = pendingPreferences.begin("rain-gauge", false);
  if (nvsOpen) {
    persistedRainLockActive = pendingPreferences.getBool("rain_lock", false);
    if (resetWithWetContact) {
      setPersistedRainLock(true);
    }
    refreshPendingQueue();
    migrateLegacyPendingSensorMessage();
  } else {
    Serial.println("Could not open NVS; using RTC RAM for pending reports.");
  }
  // Wake source identifies the trigger. Reset reason determines whether
  // the retained deadline is safe to interpret against the current clock.
  if (!resumedFromSleep ||
      rainDryDeadlineUs > cleaningSampleRtcUs + RAIN_LOCK_US) {
    const bool hadRainLock = rainDryDeadlineUs != 0 || persistedRainLockActive ||
                             resetWithWetContact || !nvsOpen;
    rainDryDeadlineUs = hadRainLock ? cleaningSampleRtcUs + RAIN_LOCK_US : 0;
  } else if (rainDryDeadlineUs == 0 && persistedRainLockActive) {
    rainDryDeadlineUs = cleaningSampleRtcUs + RAIN_LOCK_US;
  }

  // Translate the retained deadline to this boot's ISR clock, including a
  // lockout that ended between the wake event and input initialization.
  const uint64_t bootRtcUs = cleaningSampleRtcUs >= sampleUs
                                 ? cleaningSampleRtcUs - sampleUs : 0;
  portENTER_CRITICAL(&rainInterruptMux);
  cleaningAllowedFromUs = rainDryDeadlineUs > bootRtcUs
                              ? rainDryDeadlineUs - bootRtcUs : 0;
  if (cameraRequestUs < cleaningAllowedFromUs) {
    cameraRequestUs = 0;
  }
  portEXIT_CRITICAL(&rainInterruptMux);
  drainRainGaugeInterrupts();
  updateRainDryLock();
  handleCleaningInputs();

  const uint64_t nowUs = esp_rtc_get_time_us();
  if (transmissionNextRetryUs > nowUs + SENSOR_SLEEP_RETRY_MS * RTC_US_PER_MS) {
    transmissionNextRetryUs = 0;
  }
  if (!hasPendingMeasurements()) {
    transmissionNextRetryUs = 0;
  }
  tryDeepSleep();
  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, SERIAL_BAUD);
  set_text_message_callback(onMeshText);
  if (hasPendingMeasurements() && transmissionNextRetryUs <= nowUs) {
    updateTransmission();
  } else {
    requestNodeReport();
  }
}

void loop() {
  drainRainGaugeInterrupts();
  handleCleaningInputs();
  updateRainDryLock();
  pollMeshtastic();
  updateTransmission();

  const unsigned long now = millis();
  updateCleaning(now);
  updateLed(now);
  if (now - lastReportAt >= REPORT_INTERVAL_MS) {
    lastReportAt = now;
    printRainReport();
  }

  tryDeepSleep();
}
