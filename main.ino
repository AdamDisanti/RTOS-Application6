/* --------------------------------------------------------------
  Application: 06
  Adam Disanti
  Real Time Systems - Sp 2026
  Aerospace/Defense Theme: Pilot Hazard Response System

  Company context:
  - Orlando aerospace/defense style prototype
  - System for a pilot-assist / simulator hazard monitor
  - Goal: prove the hard override path meets its deadline in Wokwi

  Hardware mapping:
  - Potentiometer = Hazard Intensity Input
  - Pushbutton = Pilot Override Input
  - Green LED = System Heartbeat
  - Red LED = Threat / Alarm Indicator

  Logic analyzer mapping:
  - D0 = GPIO18 (Pilot Override Input)
  - D1 = GPIO2  (Heartbeat LED)
  - D2 = GPIO4  (Threat LED)
  - D3 = GPIO21 (ISR Strobe)
  - D4 = GPIO22 (Hard Override Response Strobe)
  - D5 = GPIO23 (Variable Load Task Strobe)
  ---------------------------------------------------------------*/

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// -------------------- Demo Output Control --------------------
#define DEMO_MODE 0

#if DEMO_MODE
#define TELEMETRY_PERIOD_MS 1000   // slow, readable
#define ENABLE_SENSOR_DEBUG 0
#else
//#define TELEMETRY_PERIOD_MS 250
#define ENABLE_SENSOR_DEBUG 1
#endif

// -------------------- Pins --------------------
#define HEARTBEAT_LED_PIN       2    // D1 on logic analyzer
#define THREAT_LED_PIN          4    // D2 on logic analyzer
#define OVERRIDE_BUTTON_PIN     18   // D0 on logic analyzer
#define HAZARD_INPUT_PIN        34

// Debug timing strobes for logic analyzer
#define ISR_STROBE_PIN          21   // D3
#define RESPONSE_STROBE_PIN     22   // D4
#define VARIABLE_STROBE_PIN     23   // D5

// -------------------- Timing --------------------
// Heartbeat Task
// Period: 500ms ON / 500ms OFF
// Deadline: 500ms per toggle
// Classification: S (Soft)
// Consequence of miss: reduced system-liveness visibility
#define HEARTBEAT_ON_MS             500
#define HEARTBEAT_OFF_MS            500

// Sensor Acquisition Task
// Period: 20ms
// Deadline: 20ms
// Classification: S (Soft)
// Consequence of miss: stale hazard samples / degraded responsiveness
#define SENSOR_PERIOD_MS            20

// Telemetry Task
// Period: 250ms
// Deadline: 250ms
// Classification: S (Soft)
// Consequence of miss: delayed debug / telemetry visibility
#define TELEMETRY_PERIOD_MS         250

// Threat Alarm Task
// Aperiodic task driven by counting semaphore
// Deadline target: <= 40ms
// Classification: S (Soft)
// Consequence of miss: delayed alert indication, but not immediate system failure
#define THREAT_BLINK_ON_MS          80
#define THREAT_BLINK_OFF_MS         80

// Button debounce window
#define BUTTON_DEBOUNCE_US          150000UL   // 50 ms

// Hard Override Response Task
// Hard deadline target: <= 5000us from ISR trigger to response strobe
// Classification: H (Hard)
// Consequence of miss: override command would not be handled in time
#define HARD_RESPONSE_DEADLINE_US   5000UL

// -------------------- Hazard thresholds --------------------
#define HAZARD_ALERT_THRESHOLD_RAW  3000
#define HAZARD_HIGH_THRESHOLD_RAW   3600

// -------------------- Scheduling --------------------
#define CORE_RT                     1
#define CORE_AUX                    0

#define PRIO_HEARTBEAT              1
#define PRIO_ASSESSMENT             2
#define PRIO_SENSOR                 3
#define PRIO_TELEMETRY              2
#define PRIO_THREAT                 3
#define PRIO_OVERRIDE               4

// -------------------- Queue/Sem sizing --------------------
#define SENSOR_QUEUE_LENGTH         32
#define THREAT_EVENT_MAX_COUNT      12

// -------------------- Types --------------------
typedef struct {
  uint32_t t_ms;
  int raw;
  uint8_t severity;
} HazardSample_t;

// -------------------- Shared state --------------------
// Flag to mark telemetry data dirty
static volatile bool g_telemetryDirty = true;

// Protected by mutex
static int g_latestHazardRaw = 0;
static uint8_t g_latestSeverity = 0;

static bool g_defenseModeEnabled = false;   // true = shielded / suppress threat LED
static bool g_threatLedLatched = false;

static uint32_t g_alertCount = 0;
static uint32_t g_overrideCount = 0;
static uint32_t g_queueDrops = 0;
static uint32_t g_queueHighWater = 0;

static uint32_t g_lastButtonIsrUs = 0;
static uint32_t g_lastOverrideLatencyUs = 0;
static uint32_t g_worstOverrideLatencyUs = 0;

static char g_lastEvent[64] = "BOOT";

// -------------------- Sync objects --------------------
static SemaphoreHandle_t xOverrideSem = NULL;   // Binary sem from ISR
static SemaphoreHandle_t xThreatSem   = NULL;   // Counting sem from hazard crossings
static SemaphoreHandle_t xStateMutex  = NULL;   // Protect serial/shared state
static QueueHandle_t xHazardQueue     = NULL;   // Internal channel for samples

// -------------------- Helper functions --------------------
static inline uint32_t ticks_to_ms(TickType_t ticks) {
  return (uint32_t)(ticks * portTICK_PERIOD_MS);
}

static uint8_t classify_severity(int raw) {
  if (raw >= HAZARD_HIGH_THRESHOLD_RAW) {
    return 3;
  } else if (raw >= HAZARD_ALERT_THRESHOLD_RAW) {
    return 2;
  } else if (raw >= 1500) {
    return 1;
  } else {
    return 0;
  }
}

static void safe_printf(const char *fmt, ...) {
  char buffer[256];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (xStateMutex == NULL) {
    Serial.print(buffer);
    return;
  }

  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.print(buffer);
    xSemaphoreGive(xStateMutex);
  }
}

static void set_last_event_text(const char *text) {
  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    strncpy(g_lastEvent, text, sizeof(g_lastEvent) - 1);
    g_lastEvent[sizeof(g_lastEvent) - 1] = '\0';
    xSemaphoreGive(xStateMutex);
  }
}

static void set_latest_hazard(int raw, uint8_t sev) {
  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_latestHazardRaw = raw;
    g_latestSeverity = sev;
    xSemaphoreGive(xStateMutex);
  }
}

static void note_queue_drop(void) {
  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_queueDrops++;
    xSemaphoreGive(xStateMutex);
  }
}

static void note_queue_highwater(void) {
  uint32_t nowWaiting = uxQueueMessagesWaiting(xHazardQueue);

  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (nowWaiting > g_queueHighWater) {
      g_queueHighWater = nowWaiting;
    }
    xSemaphoreGive(xStateMutex);
  }
}

static uint32_t increment_alert_count(void) {
  uint32_t count = 0;

  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_alertCount++;
    count = g_alertCount;
    xSemaphoreGive(xStateMutex);
  }

  return count;
}

static uint32_t increment_override_count(void) {
  uint32_t count = 0;

  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_overrideCount++;
    count = g_overrideCount;
    xSemaphoreGive(xStateMutex);
  }

  return count;
}

static void note_override_latency(uint32_t latencyUs) {
  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_lastOverrideLatencyUs = latencyUs;
    if (latencyUs > g_worstOverrideLatencyUs) {
      g_worstOverrideLatencyUs = latencyUs;
    }
    xSemaphoreGive(xStateMutex);
  }
}

static void toggle_defense_mode(void) {
  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    g_defenseModeEnabled = !g_defenseModeEnabled;
    xSemaphoreGive(xStateMutex);
  }
}

static bool get_defense_mode(void) {
  bool enabled = false;

  if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    enabled = g_defenseModeEnabled;
    xSemaphoreGive(xStateMutex);
  }

  return enabled;
}

static void pulse_response_strobe(void) {
  digitalWrite(RESPONSE_STROBE_PIN, HIGH);
  delayMicroseconds(20);
  digitalWrite(RESPONSE_STROBE_PIN, LOW);
}

static void blink_threat_led_once(void) {
  digitalWrite(THREAT_LED_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(THREAT_BLINK_ON_MS));

  digitalWrite(THREAT_LED_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(THREAT_BLINK_OFF_MS));
}

// -------------------- ISR --------------------
// Pilot Override ISR
// Hard event source
void IRAM_ATTR override_button_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint32_t nowUs = micros();

  // Debounce directly in ISR
  if ((nowUs - g_lastButtonIsrUs) < BUTTON_DEBOUNCE_US) {
    return;
  }

  g_lastButtonIsrUs = nowUs;

  // Mark ISR timing on analyzer
  digitalWrite(ISR_STROBE_PIN, HIGH);

  xSemaphoreGiveFromISR(xOverrideSem, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -------------------- Tasks --------------------

// Heartbeat Task
// Period: 500ms ON / 500ms OFF
// Deadline: 500ms per toggle
// Classification: S (Soft)
void heartbeat_task(void *pvParameters) {
  while (1) {
    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_ON_MS));

    digitalWrite(HEARTBEAT_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_OFF_MS));
  }

  vTaskDelete(NULL);
}

// Sensor Acquisition Task
// Period: 20ms
// Deadline: 20ms
// Classification: S (Soft)
void sensor_acquisition_task(void *pvParameters) {
  const TickType_t periodTicks = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  bool wasAboveThreshold = false;
  uint32_t sampleCount = 0;

  while (1) {
    int raw = analogRead(HAZARD_INPUT_PIN);
    uint8_t sev = classify_severity(raw);
    bool aboveThreshold = (raw >= HAZARD_ALERT_THRESHOLD_RAW);

    set_latest_hazard(raw, sev);

    HazardSample_t sample;
    sample.t_ms = ticks_to_ms(xTaskGetTickCount());
    sample.raw = raw;
    sample.severity = sev;

    if (xQueueSend(xHazardQueue, &sample, 0) != pdTRUE) {
      note_queue_drop();
    } else {
      note_queue_highwater();
    }

    // Threshold crossing event -> counting semaphore
    if (aboveThreshold && !wasAboveThreshold) {
      if (xSemaphoreGive(xThreatSem) == pdTRUE) {
        UBaseType_t pending = uxSemaphoreGetCount(xThreatSem);

        safe_printf("[SENSOR] t=%lums \nraw=%d \nthreshold=%d \nhazard crossing detected \npending_threat_events=%lu\n",
                    (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                    raw,
                    HAZARD_ALERT_THRESHOLD_RAW,
                    (unsigned long)pending);
      } else {
        safe_printf("[SENSOR] t=%lums \nraw=%d \nwarning: threat counting semaphore full\n",
                    (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                    raw);
      }
    }

    wasAboveThreshold = aboveThreshold;

    if ((sampleCount % 25) == 0) {
#if ENABLE_SENSOR_DEBUG
      safe_printf("[SENSOR DEBUG] raw=%d \nseverity=%u \nstate=%s\n",
                  raw,
                  (unsigned)sev,
                  aboveThreshold ? "ABOVE" : "BELOW");
#endif
    }

    sampleCount++;
    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }

  vTaskDelete(NULL);
}

// Threat Assessment Task
// Event-driven from queue
// Deadline target: <= 40ms
// Classification: S (Soft)
// Variable execution time: YES
void threat_assessment_task(void *pvParameters) {
  HazardSample_t sample;

  while (1) {
    if (xQueueReceive(xHazardQueue, &sample, portMAX_DELAY) == pdTRUE) {
      digitalWrite(VARIABLE_STROBE_PIN, HIGH);

      // Tuned variable load so it is still visible, but does not drown the system
      volatile uint32_t dummy = 0;
      uint32_t loops = 0;

      switch (sample.severity) {
        case 0: loops = 800;    break;
        case 1: loops = 4000;   break;
        case 2: loops = 10000;  break;
        case 3: loops = 18000;  break;
        default: loops = 800;   break;
      }

      for (uint32_t i = 0; i < loops; i++) {
        dummy += i;
      }

      digitalWrite(VARIABLE_STROBE_PIN, LOW);

      if (dummy == 0xFFFFFFFF) {
        safe_printf("[ASSESSMENT] impossible dummy state\n");
      }
    }
  }

  vTaskDelete(NULL);
}

// Hard Override Response Task
// Event-driven from ISR
// Hard deadline: <= 5ms from ISR trigger to response strobe
// Classification: H (Hard)
void hard_override_task(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(xOverrideSem, portMAX_DELAY) == pdTRUE) {
      uint32_t nowUs = micros();
      uint32_t latencyUs = nowUs - g_lastButtonIsrUs;

      // ISR strobe goes low when hard task begins handling the event
      digitalWrite(ISR_STROBE_PIN, LOW);

      // Hard response strobe for analyzer proof
      pulse_response_strobe();
      note_override_latency(latencyUs);

      toggle_defense_mode();
      uint32_t count = increment_override_count();

      if (latencyUs <= HARD_RESPONSE_DEADLINE_US) {
        set_last_event_text("OVERRIDE DEADLINE MET");
        g_telemetryDirty = true;
      } else {
        set_last_event_text("OVERRIDE DEADLINE MISS");
      }

      safe_printf("\n[OVERRIDE] t=%lums \ncount=%lu \nlatency_us=%lu \ndeadline_us=%lu \ndefense_mode=%s \nresult=%s\n\n",
                  (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                  (unsigned long)count,
                  (unsigned long)latencyUs,
                  (unsigned long)HARD_RESPONSE_DEADLINE_US,
                  get_defense_mode() ? "SHIELDED" : "NORMAL",
                  (latencyUs <= HARD_RESPONSE_DEADLINE_US) ? "MET" : "MISS");
    }
  }

  vTaskDelete(NULL);
}

// Threat Alarm Task
// Event-driven from counting semaphore
// Deadline target: <= 40ms
// Classification: S (Soft)
// This task is intentionally separate from the hard override path
void threat_alarm_task(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(xThreatSem, portMAX_DELAY) == pdTRUE) {
      uint32_t count = increment_alert_count();
      bool defenseMode = get_defense_mode();

      if (defenseMode) {
        set_last_event_text("THREAT SUPPRESSED");
        g_telemetryDirty = true;
        safe_printf("\n[THREAT SUPPRESSED] t=%lums \nalert=%lu \nlatest_hazard=%d \nreason=shielded mode enabled\n\n",
                    (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                    (unsigned long)count,
                    g_latestHazardRaw);
      } else {
        set_last_event_text("THREAT ALERT");
        g_telemetryDirty = true;
        safe_printf("\n[THREAT ALERT] t=%lums \nalert=%lu \nlatest_hazard=%d\n",
                    (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                    (unsigned long)count,
                    g_latestHazardRaw);

        blink_threat_led_once();
        safe_printf("[THREAT ALERT] alarm pulse complete\n\n");
      }
    }
  }

  vTaskDelete(NULL);
}

// Telemetry Task
// Period: 250ms
// Deadline: 250ms
// Classification: S (Soft)
void telemetry_task(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  const TickType_t periodTicks = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);

  while (1) {

    if (!g_telemetryDirty) {
      vTaskDelayUntil(&lastWakeTime, periodTicks);
      continue;
    }
    g_telemetryDirty = false;

    int raw = 0;
    uint8_t sev = 0;
    bool defenseMode = false;
    uint32_t alerts = 0;
    uint32_t overrides = 0;
    uint32_t drops = 0;
    uint32_t highWater = 0;
    uint32_t worstLatencyUs = 0;
    char lastEvent[64] = {0};

    if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      raw = g_latestHazardRaw;
      sev = g_latestSeverity;
      defenseMode = g_defenseModeEnabled;
      alerts = g_alertCount;
      overrides = g_overrideCount;
      drops = g_queueDrops;
      highWater = g_queueHighWater;
      worstLatencyUs = g_worstOverrideLatencyUs;
      strncpy(lastEvent, g_lastEvent, sizeof(lastEvent) - 1);
      lastEvent[sizeof(lastEvent) - 1] = '\0';
      xSemaphoreGive(xStateMutex);
    }

    safe_printf("[TELEMETRY] t=%lums \nraw=%d \nseverity=%u \ndefense_mode=%s \nalerts=%lu \noverrides=%lu \nqueue_high_water=%lu \nqueue_drops=%lu \nworst_override_latency_us=%lu \nlast_event=%s\n",
                (unsigned long)ticks_to_ms(xTaskGetTickCount()),
                raw,
                (unsigned)sev,
                defenseMode ? "SHIELDED" : "NORMAL",
                (unsigned long)alerts,
                (unsigned long)overrides,
                (unsigned long)highWater,
                (unsigned long)drops,
                (unsigned long)worstLatencyUs,
                lastEvent);

    vTaskDelayUntil(&lastWakeTime, periodTicks);
  }

  vTaskDelete(NULL);
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);

  // Create synchronization primitives
  xOverrideSem = xSemaphoreCreateBinary();
  xThreatSem = xSemaphoreCreateCounting(THREAT_EVENT_MAX_COUNT, 0);
  xStateMutex = xSemaphoreCreateMutex();
  xHazardQueue = xQueueCreate(SENSOR_QUEUE_LENGTH, sizeof(HazardSample_t));

  configASSERT(xOverrideSem);
  configASSERT(xThreatSem);
  configASSERT(xStateMutex);
  configASSERT(xHazardQueue);

  // Initialize hardware pins
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  pinMode(THREAT_LED_PIN, OUTPUT);
  pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLUP);

  pinMode(ISR_STROBE_PIN, OUTPUT);
  pinMode(RESPONSE_STROBE_PIN, OUTPUT);
  pinMode(VARIABLE_STROBE_PIN, OUTPUT);

  digitalWrite(HEARTBEAT_LED_PIN, LOW);
  digitalWrite(THREAT_LED_PIN, LOW);
  digitalWrite(ISR_STROBE_PIN, LOW);
  digitalWrite(RESPONSE_STROBE_PIN, LOW);
  digitalWrite(VARIABLE_STROBE_PIN, LOW);

  analogReadResolution(12);

  // Attach ISR
  attachInterrupt(digitalPinToInterrupt(OVERRIDE_BUTTON_PIN), override_button_isr, FALLING);

  // Startup info
  safe_printf("\n--- Application 6: Aerospace/Defense Hazard Response Prototype ---\n");
  safe_printf("Pins: HEARTBEAT_LED=%d, THREAT_LED=%d, BUTTON=%d, HAZARD_INPUT=%d\n",
              HEARTBEAT_LED_PIN, THREAT_LED_PIN, OVERRIDE_BUTTON_PIN, HAZARD_INPUT_PIN);
  safe_printf("Strobes: ISR=%d, RESPONSE=%d, VARIABLE=%d\n",
              ISR_STROBE_PIN, RESPONSE_STROBE_PIN, VARIABLE_STROBE_PIN);
  safe_printf("Hard override deadline=%lu us\n", (unsigned long)HARD_RESPONSE_DEADLINE_US);
  safe_printf("Sensor queue length=%u, item size=%u bytes\n",
              (unsigned)SENSOR_QUEUE_LENGTH, (unsigned)sizeof(HazardSample_t));
  safe_printf("System ready. Use the hazard input and pilot override button to test the prototype.\n\n");

  // Create tasks
  xTaskCreatePinnedToCore(heartbeat_task, "HEARTBEAT", 2048, NULL, PRIO_HEARTBEAT, NULL, CORE_RT);
  xTaskCreatePinnedToCore(sensor_acquisition_task, "SENSOR", 4096, NULL, PRIO_SENSOR, NULL, CORE_RT);
  xTaskCreatePinnedToCore(threat_assessment_task, "ASSESS", 4096, NULL, PRIO_ASSESSMENT, NULL, CORE_RT);
  xTaskCreatePinnedToCore(threat_alarm_task, "THREAT", 4096, NULL, PRIO_THREAT, NULL, CORE_RT);
  xTaskCreatePinnedToCore(hard_override_task, "OVERRIDE", 4096, NULL, PRIO_OVERRIDE, NULL, CORE_RT);

  // Put telemetry on the auxiliary core
  xTaskCreatePinnedToCore(telemetry_task, "TELEMETRY", 4096, NULL, PRIO_TELEMETRY, NULL, CORE_AUX);
}

void loop() {
  // All real work lives in FreeRTOS tasks
  delay(1000);
}