// ============================================================
// meshtastic_davis_anemometer.ino
// Board: Seeed XIAO nRF52840
// BSP:   Seeed nRF52 Boards >= 1.1.0
// ============================================================

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>   // usa la versione integrata nel BSP Seeed
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
#include "Meshtastic.h"

// ── Pin ──────────────────────────────────────────────────────
#define WindSensorPin  2        // Anemometro (cup rotation)
#define WindVanePin    A4       // Vane analogico
#define VaneOffset     0        // Offset da nord magnetico (gradi)

// ── Meshtastic ───────────────────────────────────────────────
#define DEST_NODE      0x12ab2251
#define SERIAL_RX_PIN  7
#define SERIAL_TX_PIN  6
#define BAUD_RATE      38400

// ── SoftwareTimer (su FreeRTOS/RTC1, compatibile SoftDevice) ──
SoftwareTimer sampleTimer;

// ── Variabili anemometro ─────────────────────────────────────
volatile bool     IsSampleRequired  = false;
volatile bool     IsMeshtasticRequired  = false;
volatile uint8_t  TimerCount        = 0;
volatile uint32_t Rotations         = 0;
volatile uint32_t ContactBounceTime = 0;

float WindSpeed         = 0.0f;
float WindSpeedMean     = 0.0f;
float WindSpeedMax      = 0.0f;
float WindSpeedSum      = 0.0f;
int   WindSpeedDiv      = 0;
int   vaneValue         = 0;
int   windDirection     = 0;
int   windCalDirection  = 0;
int   lastWindDirection = 0;
char  windCompassDirection[4] = "N";

// ── Prototipi ────────────────────────────────────────────────
void isr_timer_cb(TimerHandle_t xTimerID);
void isr_rotation();
void getWindDirection();
float getKnots(float speed);



// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Attendi Serial max 3 secondi
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 3000);

  // LED spento (HIGH = off su XIAO nRF52840)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Meshtastic UART
  mt_serial_init(SERIAL_RX_PIN, SERIAL_TX_PIN, BAUD_RATE);

  // Anemometro – interrupt su fronte di discesa
  pinMode(WindSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(WindSensorPin), isr_rotation, FALLING);

  // SoftwareTimer: callback ogni 500 ms (6 tick = 3 s → campionamento)
  sampleTimer.begin(500, isr_timer_cb);
  //sampleTimer.begin(10000, isr_timer_cb);
  sampleTimer.start();

  Serial.println("Sistema avviato.");
}

// ============================================================
// LOOP
// Il loop gira solo quando FreeRTOS lo schedula. Se non c'è
// nulla da fare, l'idle hook dorme tramite sd_app_evt_wait().
// Non servono più __WFE espliciti qui.
// ============================================================
void loop() {
  while (!IsSampleRequired) {
    __WFE();  // Wait For Event: ferma la CPU fino al prossimo evento/interrupt
    __SEV();  // assicura uscita corretta in caso di eventi persi
    __WFE();
  }
  IsSampleRequired = false;

   getWindDirection();

  if (abs(windCalDirection - lastWindDirection) > 5) {
    lastWindDirection = windCalDirection;
  }

  // Lettura sicura di Rotations (disabilita interrupt brevemente)
  noInterrupts();
  uint32_t rot = Rotations;
  Rotations = 0;
  interrupts();

  // V = P * (2.25 / 2.5) = P * 0.9  →  mph -> * 0.868976 kn
  WindSpeed = rot * 0.9f * 0.868976f;
  WindSpeedDiv++;
  WindSpeedSum=WindSpeedSum+WindSpeed;
  WindSpeedMean=WindSpeedSum/WindSpeedDiv;
  if (WindSpeed > WindSpeedMax) WindSpeedMax=WindSpeed;
 
  // Log seriale
  Serial.print("WindSpeed (kn): "); Serial.print(WindSpeed);
  Serial.print("WindSpeedMean (kn): "); Serial.print(WindSpeedMean);
  Serial.print("WindSpeedMax (kn): "); Serial.print(WindSpeedMax);
  
  Serial.print(" | Dir: ");          Serial.print(windCalDirection);
  Serial.print("° (");               Serial.print(windCompassDirection);
  Serial.println(")");

  if (IsMeshtasticRequired){
  //Invia via Meshtastic
  char buf[96];
  snprintf(buf, sizeof(buf),
           "Last:%.1f Mean: %.1f Max: %.1f kn Dir:%d %s",
           WindSpeed, WindSpeedMean, WindSpeedMax, windCalDirection, windCompassDirection);
  mt_send_text(buf, DEST_NODE, 0);
  IsMeshtasticRequired=false;
  WindSpeedSum=0;
  WindSpeedDiv=0;
  WindSpeedMean=0;
  WindSpeedMax=0;
  }
}

// ============================================================
// ISR – Timer (chiamata ogni 500 ms da SoftwareTimer su RTC1)
// ============================================================
void isr_timer_cb(TimerHandle_t xTimerID) {
  (void)xTimerID;
  TimerCount++;
  if (TimerCount >= 2) {       // 6 × 500 ms = 3 secondi
    IsSampleRequired = true;
  }
  if (TimerCount >= 30) {       // 6 × 500 ms = 3 secondi
    IsMeshtasticRequired = true;
    TimerCount = 0;
  }
}
// void isr_timer_cb(TimerHandle_t xTimerID) {
//   (void)xTimerID;
//   IsSampleRequired = true;  // diretto, nessun contatore
// }


// ============================================================
// ISR – Rotazione anemometro
// ============================================================
void isr_rotation() {
  uint32_t now = millis();
  Serial.println("Magnete");
  if ((now - ContactBounceTime) > 15) {   // debounce 15 ms
    Rotations++;
    ContactBounceTime = now;
  }
}

// ============================================================
// Direzione vento
// ============================================================
void getWindDirection() {
  vaneValue        = analogRead(WindVanePin);
  windDirection    = map(vaneValue, 0, 1023, 0, 360);
  windCalDirection = windDirection + VaneOffset;

  if (windCalDirection > 360) windCalDirection -= 360;
  if (windCalDirection <   0) windCalDirection += 360;

  if      (windCalDirection <  22) strncpy(windCompassDirection, "N",  4);
  else if (windCalDirection <  67) strncpy(windCompassDirection, "NE", 4);
  else if (windCalDirection < 112) strncpy(windCompassDirection, "E",  4);
  else if (windCalDirection < 157) strncpy(windCompassDirection, "SE", 4);
  else if (windCalDirection < 212) strncpy(windCompassDirection, "S",  4);
  else if (windCalDirection < 247) strncpy(windCompassDirection, "SW", 4);
  else if (windCalDirection < 292) strncpy(windCompassDirection, "W",  4);
  else if (windCalDirection < 337) strncpy(windCompassDirection, "NW", 4);
  else                             strncpy(windCompassDirection, "N",  4);
}

// ============================================================
// Conversione mph → nodi
// ============================================================
float getKnots(float speed) {
  return speed * 0.868976f;
}
