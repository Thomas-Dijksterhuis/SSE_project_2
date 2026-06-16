#include <Arduino.h>
#include "driver/i2s.h"
#include "driver/adc.h"

// =====================================================
// Piezo ADC  —  I2S_NUM_0 in ADC_BUILT_IN mode
// =====================================================

#define PIEZO_ADC_CHANNEL  ADC1_CHANNEL_6   // GPIO34
#define ADC_I2S_PORT       I2S_NUM_0

// =====================================================
// INMP441 digital mic  —  I2S_NUM_1 RX
// =====================================================

#define MIC_I2S_PORT  I2S_NUM_1
#define MIC_WS_PIN    33
#define MIC_SCK_PIN   32
#define MIC_SD_PIN    35

// =====================================================
// UART output to slave ESP32
//
// Uses Serial2 (UART2) — hardware UART, no CPU overhead.
// Connect:
//   Master GPIO17 (TX2)  →  Slave GPIO16 (RX2)
//   Master GND           →  Slave GND          (required!)
//
// Baud rate: 921600 is reliable on ESP32 at 3.3 V with
// short wires (<30 cm). Drop to 460800 if you see
// corruption on longer runs.
//
// Frame format over UART (per BUFFER_SIZE iteration):
//   [SYNC_BYTE] [SYNC_BYTE] [n_samples:uint16_t]
//   then n_samples × { piezo:int16_t, mic:int16_t }
//   Total payload = 4 + n*4 bytes
// =====================================================

#define UART_TX_PIN    25
#define UART_RX_PIN    26   // not used on master, but Serial2.begin needs it
#define UART_BAUD      921600

#define SYNC_BYTE      0xAA  // two sync bytes mark start of each frame
                             // slave watches for 0xAA 0xAA to re-lock

// =====================================================
// Audio parameters
// =====================================================

#define SAMPLE_RATE    20000
#define BUFFER_SIZE    256           // samples per channel per frame

// =====================================================
// Piezo ADC Setup  —  I2S_NUM_0
// =====================================================

static uint16_t adc_raw[BUFFER_SIZE];
static int32_t  mic_raw[BUFFER_SIZE];
int16_t adc_offset = 0;
void setupPiezoADC() {
  i2s_config_t cfg = {
    .mode             = (i2s_mode_t)(I2S_MODE_MASTER |
                                     I2S_MODE_RX     |
                                     I2S_MODE_ADC_BUILT_IN),
    .sample_rate      = SAMPLE_RATE,
    .bits_per_sample  = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format   = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count    = 8,
    .dma_buf_len      = 128,
    .use_apll         = false,
    .fixed_mclk       = 0,
  };

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(PIEZO_ADC_CHANNEL, ADC_ATTEN_DB_11);

  esp_err_t err;
  err = i2s_driver_install(ADC_I2S_PORT, &cfg, 0, NULL);
  Serial.printf("I2S install:  %s\n", esp_err_to_name(err));

  err = i2s_set_adc_mode(ADC_UNIT_1, PIEZO_ADC_CHANNEL);
  Serial.printf("ADC mode:     %s\n", esp_err_to_name(err));

  delay(100);

  err = i2s_adc_enable(ADC_I2S_PORT);
  Serial.printf("ADC enable:   %s\n", esp_err_to_name(err));
}

// =====================================================
// INMP441 Mic Setup  —  I2S_NUM_1
// =====================================================

void setupMic() {
  i2s_config_t cfg = {
    .mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate      = SAMPLE_RATE,
    .bits_per_sample  = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format   = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count    = 8,
    .dma_buf_len      = 128,
    .use_apll         = false,   // cleaner clock for the digital mic
    .fixed_mclk       = 0,
  };

  i2s_pin_config_t pins = {
    .mck_io_num   = I2S_PIN_NO_CHANGE,
    .bck_io_num   = MIC_SCK_PIN,
    .ws_io_num    = MIC_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = MIC_SD_PIN,
  };

  esp_err_t err;
  err = i2s_driver_install(MIC_I2S_PORT, &cfg, 0, NULL);
  Serial.printf("Mic I2S Install: %s\n", esp_err_to_name(err));

  err = i2s_set_pin(MIC_I2S_PORT, &pins);
  Serial.printf("Mic I2S Pins:    %s\n", esp_err_to_name(err));
}

void audioTask(void *) {

  while(true) {
      // --- 1. Read piezo samples via ADC I2S DMA ---
    size_t adc_bytes = 0;
    i2s_read(ADC_I2S_PORT, adc_raw, sizeof(adc_raw), &adc_bytes, portMAX_DELAY);

    // --- 2. Read mic samples via I2S DMA ---
    size_t mic_bytes = 0;
    i2s_read(MIC_I2S_PORT,
            mic_raw,
            sizeof(mic_raw),
            &mic_bytes,
            portMAX_DELAY);

    // --- 3. Determine how many complete samples we got ---
    size_t n = min((size_t)BUFFER_SIZE,
                  min(adc_bytes / sizeof(uint16_t),
                      mic_bytes / sizeof(int32_t)));

    // --- 4. Send UART frame ---
    //
    // Format:
    //   Byte 0-1 : 0xAA 0xAA  (sync marker)
    //   Byte 2-3 : n as uint16_t little-endian  (sample count)
    //   Byte 4.. : interleaved int16_t pairs [piezo, mic] × n
    //
    // The slave reads sync bytes first to lock onto frame boundaries,
    // then reads the count, then reads count×4 bytes of audio data.

    // Sync + count header
    uint8_t  header[4];
    uint16_t n16 = (uint16_t)n;
    header[0] = SYNC_BYTE;
    header[1] = SYNC_BYTE;
    header[2] = (uint8_t)(n16 & 0xFF);
    header[3] = (uint8_t)(n16 >> 8);
    Serial2.write(header, 4);

    // Interleaved audio payload — send in one shot to minimise gaps
    // Build into a small stack buffer and write once per frame.
    // 256 samples × 2 channels × 2 bytes = 1024 bytes — fits on stack fine.
    int16_t payload[BUFFER_SIZE * 2];

    for (int i = 0; i < n; i++) {
        payload[i * 2]     = (int16_t)((adc_raw[i] & 0x0FFF) - adc_offset);
        payload[i * 2 + 1] = (int16_t)(mic_raw[i] >> 16);
        // Serial.print("ADC:");
        // Serial.print(payload[i * 2] );
        // Serial.print(", MIC:");
        // Serial.println(payload[i * 2 + 1]);
    }


    Serial2.write((uint8_t*)payload, n * 2 * sizeof(int16_t));
  }
}

// =====================================================
// Setup
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");

  // UART to slave — Serial2, hardware UART2
  Serial2.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  setupPiezoADC();
  setupMic();

  i2s_adc_disable(ADC_I2S_PORT);
  delay(10);
  i2s_adc_enable(ADC_I2S_PORT);

  size_t bytes = 0;
  i2s_read(ADC_I2S_PORT, adc_raw, sizeof(adc_raw), &bytes, pdMS_TO_TICKS(200)); // flush
  i2s_read(ADC_I2S_PORT, adc_raw, sizeof(adc_raw), &bytes, pdMS_TO_TICKS(200)); // calibrate
  long sum = 0; int valid = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if ((adc_raw[i] >> 12) == (int)PIEZO_ADC_CHANNEL) {
      sum += (adc_raw[i] & 0x0FFF);
      valid++;
    }
  }
  adc_offset = valid > 0 ? (int16_t)(sum / valid) : 2048;
  Serial.printf("ADC offset: %d\n", adc_offset);

  Serial.println("=============================");
  Serial.println("ESP32 UART master ready");
  Serial.printf ("Sample rate : %d Hz\n", SAMPLE_RATE);
  Serial.printf ("Buffer size : %d samples\n", BUFFER_SIZE);
  Serial.printf ("UART baud   : %d\n", UART_BAUD);
  Serial.printf ("Bytes/frame : %d\n", 4 + BUFFER_SIZE * 4);
  Serial.println("=============================");

  xTaskCreatePinnedToCore(audioTask, "audio", 4096, NULL, 24, NULL, 1);
}

// =====================================================
// Loop
// =====================================================


void loop() {
}
