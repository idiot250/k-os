#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"

#include "kos_config.h"

#define TAG "K-OS"
#define SAMPLE_RATE 22050
#define AUDIO_BLOCK_FRAMES 256
#define TWO_PI 6.2831853071795864769f
#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

static i2s_chan_handle_t s_i2s_tx;
static adc_oneshot_unit_handle_t s_adc1;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

typedef enum { MODE_PLAY = 0, MODE_GENERATIVE = 1 } kos_mode_t;
typedef enum { ENGINE_BASS = 0, ENGINE_SCREAM = 1, ENGINE_DRONE = 2, ENGINE_NOISE = 3 } kos_engine_t;
typedef enum { SCALE_PHRYGIAN = 0, SCALE_MINOR_PENTATONIC = 1, SCALE_HARMONIC_MINOR = 2, SCALE_WHOLE_TONE = 3 } kos_scale_t;

typedef struct {
    kos_mode_t mode;
    kos_engine_t engine;
    kos_scale_t scale;
    bool fx_enabled;
    int bpm;
    int pitch_degree;
    int octave;
    int trigger_id;
    int variation_id;
    float cutoff_norm;
    float intensity;
} kos_state_t;

static kos_state_t s_state = {
    .mode = MODE_GENERATIVE,
    .engine = ENGINE_BASS,
    .scale = SCALE_PHRYGIAN,
    .fx_enabled = true,
    .bpm = 124,
    .pitch_degree = 0,
    .octave = 2,
    .trigger_id = 0,
    .variation_id = 0,
    .cutoff_norm = 0.72f,
    .intensity = 0.42f,
};

typedef struct {
    float phase_a, phase_b, phase_c, lfo_phase;
    float voice_freq;
    float voice_env;
    uint32_t gate_samples;
    float kick_phase, kick_env;
    float snare_env, hat_env;
    uint32_t step_countdown;
    unsigned step;
    unsigned last_trigger_id;
    unsigned last_variation_id;
    kos_mode_t last_mode;
    uint32_t rng;
    float lowpass_state;
} synth_runtime_t;

static const char *mode_name(kos_mode_t mode)
{
    return mode == MODE_PLAY ? "PLAY" : "GENERATIVE";
}

static const char *engine_name(kos_engine_t engine)
{
    static const char *names[] = { "DIRT BASS", "SCREECH LEAD", "DRONE", "NOISE/TEXTURE" };
    return names[(unsigned)engine % ARRAY_LEN(names)];
}

static const char *scale_name(kos_scale_t scale)
{
    static const char *names[] = { "PHRYGIAN", "MINOR PENTATONIC", "HARMONIC MINOR", "WHOLE TONE" };
    return names[(unsigned)scale % ARRAY_LEN(names)];
}

static kos_state_t get_state_snapshot(void)
{
    kos_state_t copy;
    portENTER_CRITICAL(&s_state_mux);
    copy = s_state;
    portEXIT_CRITICAL(&s_state_mux);
    return copy;
}

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float phase_wrap(float phase)
{
    if (phase >= 1.0f) phase -= floorf(phase);
    if (phase < 0.0f) phase += 1.0f - floorf(phase);
    return phase;
}

static float sine_phase(float phase)
{
    return sinf(TWO_PI * phase);
}

static float saw_phase(float phase)
{
    return 2.0f * phase - 1.0f;
}

static float square_phase(float phase, float duty)
{
    return phase < duty ? 1.0f : -1.0f;
}

static float noise_sample(synth_runtime_t *rt)
{
    /* Tiny deterministic PRNG; avoids blocking/using the hardware RNG in audio. */
    rt->rng = rt->rng * 1664525u + 1013904223u;
    return ((float)((rt->rng >> 8) & 0x00FFFFFFu) / 8388607.5f) - 1.0f;
}

static float note_frequency(int degree, int octave, kos_scale_t scale)
{
    static const int phrygian[] = { 0, 1, 3, 5, 7, 8, 10 };
    static const int pentatonic[] = { 0, 3, 5, 7, 10 };
    static const int harmonic_minor[] = { 0, 2, 3, 5, 7, 8, 11 };
    static const int whole_tone[] = { 0, 2, 4, 6, 8, 10 };
    const int *notes = phrygian;
    int count = (int)ARRAY_LEN(phrygian);

    switch (scale) {
        case SCALE_MINOR_PENTATONIC: notes = pentatonic; count = (int)ARRAY_LEN(pentatonic); break;
        case SCALE_HARMONIC_MINOR: notes = harmonic_minor; count = (int)ARRAY_LEN(harmonic_minor); break;
        case SCALE_WHOLE_TONE: notes = whole_tone; count = (int)ARRAY_LEN(whole_tone); break;
        case SCALE_PHRYGIAN:
        default: break;
    }

    int wrapped_degree = degree % count;
    int degree_octaves = degree / count;
    if (wrapped_degree < 0) {
        wrapped_degree += count;
        degree_octaves -= 1;
    }
    int midi = 36 + (octave - 2) * 12 + degree_octaves * 12 + notes[wrapped_degree];
    midi = (int)clampf_local((float)midi, 24.0f, 96.0f);
    return 440.0f * powf(2.0f, ((float)midi - 69.0f) / 12.0f);
}

static void trigger_voice(synth_runtime_t *rt, float frequency, uint32_t gate_samples)
{
    rt->voice_freq = clampf_local(frequency, 30.0f, 5000.0f);
    rt->voice_env = fmaxf(rt->voice_env, 0.001f);
    rt->gate_samples = gate_samples;
}

static void trigger_step(synth_runtime_t *rt, const kos_state_t *state)
{
    /* A deliberately asymmetric 16-step phrase; variation shifts its notes. */
    static const int melody[16] = { 0, 2, 4, 1, 5, 3, 2, 6, 0, 4, 5, 2, 1, 3, 6, 4 };
    static const int bass_pattern[16] = { 0, -1, 0, -1, 3, -1, 0, -1, 5, -1, 3, -1, 2, -1, 0, -1 };
    unsigned step = rt->step & 15u;
    unsigned variation = (unsigned)state->variation_id;
    uint32_t step_samples = (uint32_t)((float)SAMPLE_RATE * 60.0f / ((float)state->bpm * 4.0f));

    /* Kick and metallic hats keep the generative mode moving even in drone mode. */
    if (step == 0 || step == 8 || (variation % 3u == 2u && step == 14)) {
        rt->kick_env = 1.0f;
        rt->kick_phase = 0.0f;
    }
    if (step == 4 || step == 12 || (variation % 4u == 3u && step == 15)) {
        rt->snare_env = 0.9f;
    }
    if ((step % 2u) == 0u || (variation % 2u == 1u && step % 4u == 3u)) {
        rt->hat_env = 0.75f;
    }

    bool play_note = true;
    int degree = melody[(step + (variation * 3u)) & 15u];
    if (state->engine == ENGINE_BASS) {
        play_note = (bass_pattern[step] >= 0);
        if (play_note) degree = bass_pattern[step] + (int)(variation % 3u);
    } else if (state->engine == ENGINE_SCREAM) {
        play_note = (step % 2u == 0u) || (variation % 2u == 1u && step % 4u == 3u);
        degree += (int)(variation % 4u);
    } else if (state->engine == ENGINE_DRONE) {
        play_note = (step == 0 || step == 8 || (variation % 2u == 1u && step == 12));
        degree = (variation % 2u) ? 3 : 0;
    } else { /* NOISE/TEXTURE */
        play_note = (step % 4u == 0u) || (variation % 3u == 1u && step % 4u == 2u);
        degree += 7;
    }

    if (play_note) {
        float f = note_frequency(degree, state->octave, state->scale);
        if (state->engine == ENGINE_BASS) f *= 0.5f;
        if (state->engine == ENGINE_SCREAM) f *= 1.5f;
        trigger_voice(rt, f, (uint32_t)((float)step_samples * 0.68f));
    }
}

static float render_voice_sample(synth_runtime_t *rt, const kos_state_t *state, float intensity)
{
    float f = rt->voice_freq;
    float lfo_hz = 0.13f + intensity * 7.2f;
    rt->lfo_phase = phase_wrap(rt->lfo_phase + lfo_hz / (float)SAMPLE_RATE);
    float lfo = sine_phase(rt->lfo_phase);
    float s = 0.0f;

    switch (state->engine) {
        case ENGINE_BASS: {
            float sub_f = f * 0.5f;
            rt->phase_a = phase_wrap(rt->phase_a + f / (float)SAMPLE_RATE);
            rt->phase_b = phase_wrap(rt->phase_b + sub_f / (float)SAMPLE_RATE);
            s = 0.48f * saw_phase(rt->phase_a) + 0.52f * sine_phase(rt->phase_b);
            break;
        }
        case ENGINE_SCREAM: {
            float mod_f = f * (1.0f + 0.035f * intensity * sine_phase(rt->lfo_phase * 0.37f));
            rt->phase_a = phase_wrap(rt->phase_a + mod_f / (float)SAMPLE_RATE);
            rt->phase_b = phase_wrap(rt->phase_b + (f * 1.005f) / (float)SAMPLE_RATE);
            float duty = 0.18f + 0.25f * (0.5f + 0.5f * lfo);
            s = 0.62f * saw_phase(rt->phase_a) + 0.38f * square_phase(rt->phase_b, duty);
            break;
        }
        case ENGINE_DRONE: {
            rt->phase_a = phase_wrap(rt->phase_a + f / (float)SAMPLE_RATE);
            rt->phase_b = phase_wrap(rt->phase_b + (f * 1.006f) / (float)SAMPLE_RATE);
            rt->phase_c = phase_wrap(rt->phase_c + (f * 0.5f) / (float)SAMPLE_RATE);
            s = 0.43f * sine_phase(rt->phase_a) + 0.36f * sine_phase(rt->phase_b) + 0.21f * sine_phase(rt->phase_c);
            s *= 0.7f + 0.3f * lfo;
            break;
        }
        case ENGINE_NOISE:
        default: {
            rt->phase_a = phase_wrap(rt->phase_a + (f * (1.0f + intensity * 0.4f)) / (float)SAMPLE_RATE);
            float squeal = square_phase(rt->phase_a, 0.12f + 0.22f * (0.5f + 0.5f * lfo));
            s = 0.68f * noise_sample(rt) + 0.32f * squeal;
            break;
        }
    }
    (void)state;
    return s * rt->voice_env;
}

static void render_block(int16_t *output, const kos_state_t *state, synth_runtime_t *rt)
{
    float cutoff_hz = 120.0f * powf(60.0f, clampf_local(state->cutoff_norm, 0.0f, 1.0f));
    cutoff_hz = clampf_local(cutoff_hz, 120.0f, 7200.0f);
    float filter_alpha = 1.0f - expf(-TWO_PI * cutoff_hz / (float)SAMPLE_RATE);
    float intensity = clampf_local(state->intensity, 0.0f, 1.0f);

    if (state->mode != rt->last_mode) {
        rt->step_countdown = 0;
        rt->last_mode = state->mode;
    }
    if (state->mode == MODE_PLAY && (unsigned)state->trigger_id != rt->last_trigger_id) {
        rt->last_trigger_id = (unsigned)state->trigger_id;
        float f = note_frequency(state->pitch_degree, state->octave, state->scale);
        trigger_voice(rt, f, SAMPLE_RATE / 5u);
    }
    if ((unsigned)state->variation_id != rt->last_variation_id) {
        rt->last_variation_id = (unsigned)state->variation_id;
        rt->step = (rt->step + 4u) & 15u;
        rt->step_countdown = 0;
    }

    for (int i = 0; i < AUDIO_BLOCK_FRAMES; ++i) {
        if (state->mode == MODE_GENERATIVE) {
            if (rt->step_countdown == 0u) {
                trigger_step(rt, state);
                uint32_t step_samples = (uint32_t)((float)SAMPLE_RATE * 60.0f / ((float)state->bpm * 4.0f));
                rt->step_countdown = step_samples > 0u ? step_samples : 1u;
                rt->step = (rt->step + 1u) & 15u;
            }
            rt->step_countdown--;
        }

        if (rt->gate_samples > 0u) {
            rt->voice_env += (1.0f - rt->voice_env) * 0.0022f;
            rt->gate_samples--;
        } else {
            rt->voice_env *= 0.99978f;
            if (rt->voice_env < 0.00008f) rt->voice_env = 0.0f;
        }

        float sample = render_voice_sample(rt, state, intensity);

        /* Kick: downward-dropping sine pitch gives a compact electronic thump. */
        if (rt->kick_env > 0.0001f) {
            float kick_hz = 42.0f + 105.0f * rt->kick_env;
            rt->kick_phase = phase_wrap(rt->kick_phase + kick_hz / (float)SAMPLE_RATE);
            sample += 0.85f * sine_phase(rt->kick_phase) * rt->kick_env;
            rt->kick_env *= 0.99955f;
        }
        if (rt->snare_env > 0.0001f) {
            sample += (0.30f * noise_sample(rt) + 0.12f * sine_phase(rt->phase_c)) * rt->snare_env;
            rt->snare_env *= 0.9992f;
            rt->phase_c = phase_wrap(rt->phase_c + 180.0f / (float)SAMPLE_RATE);
        }
        if (rt->hat_env > 0.0001f) {
            sample += 0.19f * noise_sample(rt) * rt->hat_env;
            rt->hat_env *= 0.9978f;
        }

        /* One-pole resonant-ish tone control; drive adds grit and aliasy texture. */
        rt->lowpass_state += filter_alpha * (sample - rt->lowpass_state);
        float shaped = rt->lowpass_state;
        if (state->fx_enabled) {
            float drive = 1.3f + intensity * 11.0f;
            shaped = tanhf(shaped * drive) / tanhf(drive);
            float levels = 112.0f - intensity * 88.0f;
            shaped = roundf(shaped * levels) / levels;
        }

        /* Conservative fixed output level; there is no dedicated master-volume knob. */
        shaped *= 0.62f;
        shaped = clampf_local(shaped, -0.98f, 0.98f);
        int16_t pcm = (int16_t)(shaped * 32767.0f);
        output[i * 2] = pcm;
        output[i * 2 + 1] = pcm;
    }
}

static void audio_task(void *arg)
{
    (void)arg;
    int16_t buffer[AUDIO_BLOCK_FRAMES * 2];
    synth_runtime_t rt = {
        .phase_a = 0.0f, .phase_b = 0.0f, .phase_c = 0.0f, .lfo_phase = 0.0f,
        .voice_freq = 110.0f, .voice_env = 0.0f, .gate_samples = 0,
        .kick_phase = 0.0f, .kick_env = 0.0f, .snare_env = 0.0f, .hat_env = 0.0f,
        .step_countdown = 0, .step = 0, .last_trigger_id = 0,
        .last_variation_id = 0, .last_mode = MODE_PLAY,
        .rng = 0x91E10DA5u, .lowpass_state = 0.0f,
    };

    while (true) {
        kos_state_t state = get_state_snapshot();
        render_block(buffer, &state, &rt);
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(s_i2s_tx, buffer, sizeof(buffer), &bytes_written, 1000);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "I2S write failed: %s", esp_err_to_name(err));
        }
    }
}

static void configure_i2s(void)
{
    i2s_chan_config_t channel_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_cfg.dma_desc_num = 6;
    channel_cfg.dma_frame_num = AUDIO_BLOCK_FRAMES;
    ESP_ERROR_CHECK(i2s_new_channel(&channel_cfg, &s_i2s_tx, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = KOS_I2S_BCLK_GPIO,
            .ws = KOS_I2S_LRCK_GPIO,
            .dout = KOS_I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));
}

static void configure_gpio_inputs(void)
{
    uint64_t mask = (1ULL << KOS_ENCODER_CLK_GPIO) |
                    (1ULL << KOS_ENCODER_DT_GPIO) |
                    (1ULL << KOS_ENCODER_SW_GPIO) |
                    (1ULL << KOS_BUTTON_MODE_GPIO) |
                    (1ULL << KOS_BUTTON_ENGINE_GPIO) |
                    (1ULL << KOS_BUTTON_VARIATION_GPIO) |
                    (1ULL << KOS_BUTTON_FX_GPIO);
    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

static void configure_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc1));

    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, KOS_POT_CUTOFF_CHANNEL, &channel_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, KOS_POT_INTENSITY_CHANNEL, &channel_cfg));
}

static bool pressed_debounced(gpio_num_t pin, bool *last_raw, TickType_t *changed_at, bool *stable_state)
{
    bool raw = gpio_get_level(pin) == 0; /* pressed = LOW with internal pull-up */
    TickType_t now = xTaskGetTickCount();
    if (raw != *last_raw) {
        *last_raw = raw;
        *changed_at = now;
    }
    if ((now - *changed_at) >= pdMS_TO_TICKS(28) && raw != *stable_state) {
        *stable_state = raw;
        if (raw) return true; /* press edge only; release is ignored */
    }
    return false;
}

static void log_state(const char *reason)
{
    kos_state_t s = get_state_snapshot();
    ESP_LOGI(TAG, "%s | mode=%s | engine=%s | scale=%s | BPM=%d | FX=%s | pitch degree=%d octave=%d | cutoff=%.2f intensity=%.2f",
             reason, mode_name(s.mode), engine_name(s.engine), scale_name(s.scale), s.bpm,
             s.fx_enabled ? "ON" : "OFF", s.pitch_degree, s.octave, s.cutoff_norm, s.intensity);
}

static void apply_encoder_turn(int direction)
{
    portENTER_CRITICAL(&s_state_mux);
    if (s_state.mode == MODE_PLAY) {
        s_state.pitch_degree += direction;
        if (s_state.pitch_degree < 0) s_state.pitch_degree = 15;
        if (s_state.pitch_degree > 15) s_state.pitch_degree = 0;
    } else {
        s_state.bpm += direction * 2;
        if (s_state.bpm < 70) s_state.bpm = 70;
        if (s_state.bpm > 170) s_state.bpm = 170;
    }
    portEXIT_CRITICAL(&s_state_mux);
}

static void encoder_click(void)
{
    portENTER_CRITICAL(&s_state_mux);
    if (s_state.mode == MODE_PLAY) {
        s_state.trigger_id++;
    } else {
        s_state.scale = (kos_scale_t)(((unsigned)s_state.scale + 1u) % 4u);
        s_state.variation_id++;
    }
    portEXIT_CRITICAL(&s_state_mux);
    log_state("Encoder press");
}

static void control_task(void *arg)
{
    (void)arg;
    bool last_raw[4] = { false, false, false, false };
    bool stable[4] = { false, false, false, false };
    TickType_t changed_at[4] = { 0, 0, 0, 0 };
    bool enc_last_raw = false, enc_stable = false;
    TickType_t enc_changed_at = 0;
    int previous_encoder = ((gpio_get_level(KOS_ENCODER_CLK_GPIO) ? 1 : 0) << 1) |
                           (gpio_get_level(KOS_ENCODER_DT_GPIO) ? 1 : 0);
    int encoder_accumulator = 0;
    float cutoff_filtered = s_state.cutoff_norm;
    float intensity_filtered = s_state.intensity;
    static const int8_t enc_table[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
    const gpio_num_t buttons[4] = { KOS_BUTTON_MODE_GPIO, KOS_BUTTON_ENGINE_GPIO,
                                    KOS_BUTTON_VARIATION_GPIO, KOS_BUTTON_FX_GPIO };

    ESP_LOGI(TAG, "Controls: encoder=note/BPM, encoder click=trigger/scale; B1=mode, B2=engine, B3=variation/trigger, B4=FX.");
    ESP_LOGI(TAG, "Pot 1=cutoff; Pot 2=intensity/drive. Switches are assumed active-low (GPIO to GND when pressed).");

    while (true) {
        int raw1 = 0, raw2 = 0;
        if (adc_oneshot_read(s_adc1, KOS_POT_CUTOFF_CHANNEL, &raw1) == ESP_OK) {
            float norm = (float)raw1 / 4095.0f;
            cutoff_filtered = 0.82f * cutoff_filtered + 0.18f * norm;
        }
        if (adc_oneshot_read(s_adc1, KOS_POT_INTENSITY_CHANNEL, &raw2) == ESP_OK) {
            float norm = (float)raw2 / 4095.0f;
            intensity_filtered = 0.82f * intensity_filtered + 0.18f * norm;
        }
        portENTER_CRITICAL(&s_state_mux);
        s_state.cutoff_norm = cutoff_filtered;
        s_state.intensity = intensity_filtered;
        portEXIT_CRITICAL(&s_state_mux);

        int current_encoder = ((gpio_get_level(KOS_ENCODER_CLK_GPIO) ? 1 : 0) << 1) |
                              (gpio_get_level(KOS_ENCODER_DT_GPIO) ? 1 : 0);
        int table_index = (previous_encoder << 2) | current_encoder;
        encoder_accumulator += enc_table[table_index & 0x0F];
        previous_encoder = current_encoder;
        if (encoder_accumulator >= 4) {
            apply_encoder_turn(1);
            encoder_accumulator = 0;
        } else if (encoder_accumulator <= -4) {
            apply_encoder_turn(-1);
            encoder_accumulator = 0;
        }

        if (pressed_debounced(KOS_ENCODER_SW_GPIO, &enc_last_raw, &enc_changed_at, &enc_stable)) {
            encoder_click();
        }

        for (int i = 0; i < 4; ++i) {
            if (!pressed_debounced(buttons[i], &last_raw[i], &changed_at[i], &stable[i])) continue;
            portENTER_CRITICAL(&s_state_mux);
            switch (i) {
                case 0:
                    s_state.mode = (s_state.mode == MODE_PLAY) ? MODE_GENERATIVE : MODE_PLAY;
                    break;
                case 1:
                    s_state.engine = (kos_engine_t)(((unsigned)s_state.engine + 1u) % 4u);
                    break;
                case 2:
                    if (s_state.mode == MODE_PLAY) s_state.trigger_id++;
                    else s_state.variation_id++;
                    break;
                case 3:
                    s_state.fx_enabled = !s_state.fx_enabled;
                    break;
            }
            portEXIT_CRITICAL(&s_state_mux);
            static const char *button_names[] = { "Mode button", "Engine button", "Variation/trigger button", "FX button" };
            log_state(button_names[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "K-OS starting: ESP32-S3 + PCM5102A, %d Hz stereo I2S", SAMPLE_RATE);
    configure_gpio_inputs();
    configure_adc();
    configure_i2s();

    ESP_LOGW(TAG, "Potentiometer wipers are configured for GPIO4/5 (ADC1 channels 3/4); verify these match your PCB.");
    log_state("Initial state");

    xTaskCreatePinnedToCore(audio_task, "kos_audio", 6144, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(control_task, "kos_controls", 4096, NULL, 4, NULL, 0);
}
