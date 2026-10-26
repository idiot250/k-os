# K-OS firmware (ESP-IDF)

A first complete, standalone synthesizer/sequencer firmware for the ESP32-S3 and PCM5102A I2S DAC. It produces sound internally; it does not require a computer, phone, MIDI device, network, or external audio library.

## Important hardware assumption

The conversation context preserves the I2S, encoder, and button GPIO assignments, but not the two potentiometer wiper GPIOs. `main/kos_config.h` therefore uses **GPIO4 / ADC1_CH3** and **GPIO5 / ADC1_CH4** as the best reconstruction of the earlier pin assignment. These only work if they match the actual PCB connections. If the schematic used other pins, change the `KOS_POT_*_GPIO` and `KOS_POT_*_CHANNEL` macros before flashing. GPIO4 maps to ADC1 channel 3 and GPIO5 to ADC1 channel 4 on ESP32-S3.

The button/encoder-switch code assumes each switch connects its GPIO to **GND while pressed**, using the ESP32-S3's internal pull-up. That matches the common wiring we discussed, but should be verified if a switch behaves backward or never triggers.

## Pin mapping currently configured

| Function | ESP32-S3 GPIO |
|---|---:|
| PCM5102A BCLK | 41 |
| PCM5102A LRCK / WS | 40 |
| ESP32 I2S data out -> PCM5102A DIN | 39 |
| KY-040 CLK | 6 |
| KY-040 DT | 7 |
| KY-040 SW | 15 |
| Button 1: mode | 16 |
| Button 2: engine | 17 |
| Button 3: variation / trigger | 18 |
| Button 4: FX | 8 |
| Pot 1: filter cutoff (reconstructed; verify) | 4 / ADC1_CH3 |
| Pot 2: intensity / distortion (reconstructed; verify) | 5 / ADC1_CH4 |

Power the PCM5102A module according to that exact module's specifications and connect ESP32 ground to DAC ground. The firmware expects the DAC's **DIN** pin to receive ESP32 GPIO39; it does not send audio through USB.

## Controls

- **Button 1**: switch between PLAY and GENERATIVE modes.
- **Button 2**: cycle DIRT BASS -> SCREECH LEAD -> DRONE -> NOISE/TEXTURE.
- **Button 3**: PLAY: trigger the current note. GENERATIVE: introduce a variation/fill.
- **Button 4**: toggle distortion/bit-crush processing.
- **Encoder rotation**: PLAY: change pitch degree. GENERATIVE: change tempo from 70–170 BPM.
- **Encoder press**: PLAY: trigger a note. GENERATIVE: cycle scale and introduce a variation.
- **Pot 1**: filter cutoff, low to high.
- **Pot 2**: intensity; with FX enabled it increases drive and bit-crushing.

The firmware logs state changes and the pin assumptions to the serial console. There is no display assumed in the hardware.

## Build and flash

Use ESP-IDF 5.3 or newer, with an ESP32-S3 target configured. From this directory in an ESP-IDF terminal:

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Replace `/dev/ttyACM0` with the serial port for your board. If you have already configured the project target, you can skip `set-target`. Exit the serial monitor with `Ctrl+]`.

## First power-up

1. Turn the audio system/amplifier volume down before applying power; then raise it slowly.
2. Verify the actual pot wiper pins against `main/kos_config.h` before flashing.
3. Check that the DAC module's power input is suitable for the voltage wired in your PCB.
4. If buttons trigger in reverse or not at all, check whether the switches close to GND when pressed. Change the input logic if your schematic uses a different arrangement.
5. If the audio is silent, check serial logs, shared ground, DAC power, and BCLK/LRCK/DIN wiring first.

## Current scope / caveats

This is a functional first-generation digital instrument, not a polished commercial synth. It provides four synthesized sound engines, a 16-step generative pattern with kick/snare/hat events, a low-pass tone control, drive/bit-crushing, scale selection, and tempo control. The tone is intentionally raw and digital. There is no sample playback, USB audio, external MIDI, OLED UI, or persistent preset storage.
