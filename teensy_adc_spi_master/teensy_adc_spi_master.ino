#include <driver/i2s.h>

// I2S pin definitions (matching original SPI pins)
#define I2S_BCK_PIN   26   // Was SPI_AUDIO_CLK
#define I2S_WS_PIN    25   // Was SPI_AUDIO_MOSI
#define I2S_DATA_PIN  27   // Was SPI_AUDIO_MISO

#define SAMPLE_RATE   20000
#define BUFFER_SIZE   128
#define CHANNELS      2
#define FRAME_SAMPLES (BUFFER_SIZE * CHANNELS)

#define I2S_PORT      I2S_NUM_0

int16_t samples[FRAME_SAMPLES];

float phase    = 0.0f;
float modPhase = 0.0f;

void generateWail(int16_t* buffer, int numSamples) {
  for (int i = 0; i < numSamples; i += 2) {
    float lfo  = 0.5f + 0.5f * sinf(modPhase);
    float freq = 300.0f + lfo * 1200.0f;
    float amp  = 0.3f + 0.7f * lfo;
    float val  = sinf(phase) * amp;

    phase    += 2.0f * M_PI * freq / SAMPLE_RATE;
    modPhase += 2.0f * M_PI * 0.5f / SAMPLE_RATE;

    while (phase > 2.0f * M_PI) phase -= 2.0f * M_PI;
    while (modPhase > 2.0f * M_PI) modPhase -= 2.0f * M_PI;

    buffer[i]     = (int16_t)(val * 32767.0f);  // Left channel
    buffer[i + 1] = buffer[i];                  // Right channel
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000);

  i2s_config_t i2s_config = {
    .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate          = SAMPLE_RATE,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 4,
    .dma_buf_len          = BUFFER_SIZE,
    .use_apll             = false,
    .tx_desc_auto_clear   = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num   = I2S_BCK_PIN,
    .ws_io_num    = I2S_WS_PIN,
    .data_out_num = I2S_DATA_PIN,
    .data_in_num  = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);

  Serial.println("=============================");
  Serial.println("ESP32 I2S master ready");
  Serial.println("=============================");
  Serial.print  ("Sample rate: ");
  Serial.print  (SAMPLE_RATE);
  Serial.println(" Hz");
}

void loop() {
  generateWail(samples, FRAME_SAMPLES);

  size_t bytesWritten = 0;
  i2s_write(I2S_PORT,
            samples,
            FRAME_SAMPLES * sizeof(int16_t),
            &bytesWritten,
            portMAX_DELAY);
}