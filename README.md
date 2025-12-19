# Pico BT Audio Receiver

A Bluetooth A2DP audio receiver for the Raspberry Pi Pico W, outputting high-quality audio via I2S to a PCM5102A DAC.

## Features

- **Bluetooth A2DP Sink** - Receives stereo audio from phones, computers, and other Bluetooth sources
- **SBC Codec Decoding** - Decodes standard Bluetooth audio codec
- **I2S Output** - Clean digital audio output to PCM5102A or similar DAC
- **Gapless Playback** - DMA double-buffering with chaining for uninterrupted audio
- **Ring Buffer** - 16KB buffer absorbs Bluetooth jitter for smooth playback
- **Dynamic Sample Rate** - Automatically detects and adapts to 44.1kHz or 48kHz streams
- **AVRCP Support** - Volume control from source device

## Hardware Requirements

- **Raspberry Pi Pico W**
- **PCM5102A I2S DAC Module** (or compatible)
- **3.5mm Audio Jack** or speaker/amplifier
- **Jumper wires**

## Wiring Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI PICO W                        │
│                                                                 │
│   3V3 ●──────────────────────────────────────●  VCC (PCM5102A)  │
│   GND ●──────────────────────────────────────●  GND             │
│                                                                 │
│   GPIO 18 (BCK)  ●───────────────────────────●  BCK             │
│   GPIO 19 (LCK)  ●───────────────────────────●  LCK (LRCK)      │
│   GPIO 20 (DIN)  ●───────────────────────────●  DIN             │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

PCM5102A Module Configuration:
┌──────────────────────┐
│     PCM5102A DAC     │
│                      │
│  FMT ●───● GND       │  (I2S Standard Format)
│  XMT ●───● 3.3V      │  (MUST be HIGH - Mute OFF)
│  SCK ●───● GND       │  (Use internal clock from BCK)
│                      │
│  LOUT ●──────────────│──► Left Audio Out
│  ROUT ●──────────────│──► Right Audio Out
│                      │
└──────────────────────┘
```

### Pin Connections

| Pico W Pin | PCM5102A Pin | Description |
|------------|--------------|-------------|
| 3V3 (OUT)  | VCC          | Power supply |
| GND        | GND          | Ground |
| GPIO 18    | BCK          | Bit Clock |
| GPIO 19    | LCK (LRCK)   | Left/Right Clock (Word Select) |
| GPIO 20    | DIN          | Serial Data |

### PCM5102A Configuration Pins

| Pin | Connection | Description |
|-----|------------|-------------|
| FMT | GND        | I2S Standard format |
| XMT | **3.3V**   | **Critical: Must be HIGH for audio output** |
| SCK | GND        | Use internal clock generation |

> ⚠️ **Important:** If XMT is LOW or floating, the DAC will be muted and you won't hear any audio!

## Building

### Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) (tested with 2.0+)
- CMake 3.13+
- ARM GCC toolchain

### Build Steps

```bash
# Clone the repository
git clone https://github.com/Gil-AdB/pico-bt-audio-receiver.git
cd pico-bt-audio-receiver

# Create build directory
mkdir build && cd build

# Configure (set PICO_SDK_PATH if needed)
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk

# Build
make -j8
# or
ninja
```

### Flash

1. Hold BOOTSEL button on Pico W
2. Connect USB cable
3. Release BOOTSEL
4. Copy `pico_bt_audio.uf2` to the RPI-RP2 drive

## Usage

1. Power on the Pico W (LED will blink fast indicating discoverable mode)
2. On your phone/computer, search for Bluetooth devices
3. Connect to **"Pico Audio Receiver"**
4. Play audio - it will stream to the DAC

### LED Indicators

| Pattern | Meaning |
|---------|---------|
| Fast blink (250ms) | Discoverable, waiting for connection |
| Slow blink (1s) | Connected, streaming active |

## Architecture

```
┌──────────────┐     ┌───────────────┐     ┌──────────────────┐     ┌─────────┐
│  Bluetooth   │────▶│  SBC Decoder  │────▶│  Ring Buffer     │────▶│  I2S    │
│  A2DP Source │     │  (BTstack)    │     │  (16KB / ~180ms) │     │  Output │
└──────────────┘     └───────────────┘     └──────────────────┘     └─────────┘
                                                    │
                                           DMA IRQ pulls data
                                                    ▼
                                           ┌──────────────────┐
                                           │ DMA Double Buffer│
                                           │  A ←chain→ B     │
                                           │  (gapless swap)  │
                                           └──────────────────┘
```

## Troubleshooting

### No Audio Output

1. **Check XMT pin** - Must be connected to 3.3V (not floating!)
2. **Verify wiring** - BCK, LCK, DIN connections
3. **Check power** - DAC VCC connected to 3.3V
4. **Test with serial** - Connect USB and check debug output

### Distorted/Garbled Audio

1. Wrong sample rate - check `[AUDIO] Detected sample rate: XXXX Hz` in serial output
2. Wiring issue - verify BCK and LCK are not swapped

### Bluetooth Won't Connect

1. Power cycle the Pico W
2. Remove pairing from phone and re-pair
3. Check if another device is already connected

## License

MIT License

## Acknowledgments

- [BTstack](https://github.com/bluekitchen/btstack) - Bluetooth stack
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- [pico-extras](https://github.com/raspberrypi/pico-extras) - Audio I2S PIO program
