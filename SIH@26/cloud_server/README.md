# SIH 2026 Edge Voice Activator: Wokwi Simulation (ESP32-S3)

Wake word: **"hey comet"**. The ESP32-S3 spots the keyword locally and only talks to the cloud after a detection.

## 1. What this simulation demonstrates

The full event-driven pipeline, end to end, with real networking:

```
AudioSource (simulated) -> 32 ms DMA-style blocks -> KWS (MOCK_KWS) -> threshold 0.80
  -> MQTT event KEYWORD_DETECTED -> 1 s post-keyword capture -> MQTT audio packet
  -> cloud_server (MOCK_ASR) -> MQTT command JSON -> LED action -> back to LISTENING
```

It also shows an explicit, non-blocking state machine (`BOOT → WIFI_CONNECTING → MQTT_CONNECTING → LISTENING → KEYWORD_DETECTED → CAPTURING → UPLOADING → WAITING_FOR_ASR → COMMAND_RECEIVED → LISTENING`), Wi-Fi timeout and reconnect, MQTT reconnect with Last Will, request/response correlation (`request_id`), and rejection of a partial "distractor" pattern.

## 2. What is simulated (and clearly marked as such)

| Component | What it really is | Serial tag |
|---|---|---|
| Microphone | `SimulatedAudioSource`: a software-generated 16 kHz int16 signal (near-silence, noise + 50 Hz hum, tone patterns). **Not a microphone, not speech.** | `[MIC-SIM]` |
| Keyword spotter | `MOCK_KWS`: Goertzel tone-sequence detector. It analyses only the samples it receives, but it is **not a speech model**. | `[MOCK_KWS]` |
| ASR | `MOCK_ASR` in the cloud server: deterministic fake transcripts. | `asr_mode: MOCK_ASR` |
| Audio upload | A compact JSON **summary** of the capture (sample count, RMS, peak). No raw PCM is sent. | `SIMULATED_AUDIO_SUMMARY` |

The simulated timeline repeats every 10 s from the moment listening starts:

| Time | Signal | Expected result |
|---|---|---|
| 0.0 – 1.0 s | near-silence (±4 LSB) | nothing |
| 1.0 – 10 s | background noise ±1000 + 50 Hz hum | nothing |
| 3.0 – 3.2 s | **distractor**: 1000 Hz only | `p=0.33`, rejected |
| 6.0 – 6.6 s | **keyword test pattern**: 1000 → 1500 → 2000 Hz, 200 ms each | `p=1.00`, detected |

Press the **red button** (GPIO 14) to inject the keyword test pattern immediately.

## 3. What is real

The ESP32-S3 firmware (it is the same code you will flash), the Wi-Fi connection to `Wokwi-GUEST`, the TCP/MQTT connection to a real internet broker, the JSON on the wire, the cloud server, and the LED/GPIO behaviour.

## 4. How to open the Wokwi project

1. Go to <https://wokwi.com>, start a new project and choose an **ESP32-S3** board (Arduino template).
2. Replace the contents of `sketch.ino` and `diagram.json` with the files from this folder.
3. Add the libraries: open the **Library Manager** tab and add `PubSubClient` and `ArduinoJson`, or create a file named `libraries.txt` with this folder's content.
4. Press **▶ Start**. The Serial Monitor opens automatically.
5. Start the cloud server (section 5), which can be started before or after the simulation.

The circuit:

| GPIO | Part | Meaning |
|---|---|---|
| 4 | green LED | Wi-Fi: blinking = connecting, solid = connected |
| 5 | yellow LED | keyword detected / capturing / waiting |
| 6 | blue LED | MQTT: blinking slow = connecting, solid = connected, fast = cloud traffic |
| 7 | white LED | the "light" controlled by `TURN_ON` / `TURN_OFF` |
| 14 | red button | inject keyword test pattern |

Expected serial output (abridged):

```
[BOOT]
[BOOT] Audio source : SimulatedAudioSource (SIMULATED — synthetic test signal, not a real mic)
[BOOT] KWS backend  : MOCK_KWS (MOCK — tone-pattern detector, not a speech model)
[WIFI] Connecting...
[WIFI] Connected (IP ..., RSSI ... dBm)
[MQTT] Connecting...
[MQTT] Connected
[KWS] Listening... (backend MOCK_KWS — MOCK, not a speech model, label "hey_comet", threshold 0.80)
[MOCK_KWS] candidate: A=6 B=0 C=0 frames -> p=0.33 (reject, below threshold)
[MOCK_KWS] candidate: A=6 B=6 C=6 frames -> p=1.00 (ACCEPT)
[KWS] Keyword detected: "hey_comet" p=1.00 >= 0.80 [MOCK_KWS]
[MQTT] Published sih/device01/event: KEYWORD_DETECTED
[CAPTURE] Capturing 1000 ms of post-keyword audio (SIMULATED signal)
[MQTT] Sending audio/command
[ASR] Waiting for response (request device01-0001, timeout 10000 ms)
[ASR] Command received: TURN_ON
[CMD] Light ON (GPIO 7)
[KWS] Listening...
```

MQTT connects *before* listening (per the state machine), so it appears before the first `[KWS] Listening...`. That keeps the connection warm, which removes connect latency from every wake-word interaction. If the cloud server is not running you will see `[ASR] Timeout` after 10 s, and the device goes back to listening.

## 5. How to run the cloud server

```bash
cd cloud_server
python3 -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python server.py
```

By default both sides use `test.mosquitto.org:1883`. See `cloud_server/README.md` for configuration.

## 6. How MQTT communication works

| Topic | Direction | Payload |
|---|---|---|
| `sih/device01/status` | device → cloud | `ONLINE` (retained). The broker publishes `OFFLINE` (Last Will) if the device disappears. |
| `sih/device01/event` | device → cloud | `KEYWORD_DETECTED` (plain text) |
| `sih/device01/audio` | device → cloud | JSON audio summary (below). A real device sends binary PCM chunks (see `docs/architecture.md`). |
| `sih/device01/command` | cloud → device | JSON command (below), QoS 1 |

Audio summary (Wokwi build):
```json
{"type":"SIMULATED_AUDIO_SUMMARY","simulated":true,"device_id":"device01",
 "request_id":"device01-0001","seq":1,"sample_rate":16000,"channels":1,
 "format":"pcm_s16le","samples":16384,"duration_ms":1024,"rms":616,"peak":1299,
 "kws_backend":"MOCK_KWS","kws_label":"hey_comet","kws_probability":1,
 "audio_source":"SimulatedAudioSource","uptime_ms":9364}
```

Command:
```json
{"request_id":"device01-0001","command":"TURN_ON","text":"turn on the light",
 "asr_mode":"MOCK_ASR","simulated_audio":true}
```

The device accepts a command only while in `WAITING_FOR_ASR` and only if `request_id` matches, or is absent. Stale, unsolicited and malformed commands are logged and ignored. Supported commands: `TURN_ON`, `TURN_OFF`, `STATUS`. Anything else blinks the yellow LED 3 times.

To watch the traffic: `mosquitto_sub -h test.mosquitto.org -t 'sih/device01/#' -v`

> **Public broker warning:** anyone can read and publish on `test.mosquitto.org`. That is acceptable for tone patterns. Never send real voice audio there. To avoid clashing with other users, set a unique `DEVICE_ID` (in `sketch.ino` **and** the server's env), which changes every topic.

## 7. How to add an Edge Impulse Arduino library

1. In Edge Impulse Studio, train on **16 kHz** audio with a 1000 ms window. Use labels such as `hey_comet`, `noise`, `unknown`.
2. **Deployment → Arduino library** → download the ZIP.
3. Arduino IDE: **Sketch → Include Library → Add .ZIP Library…** (PlatformIO: unzip into `lib/`).
4. Build for **ESP32S3 Dev Module**.

## 8. How to replace MOCK_KWS with the real model (REPLACEMENT POINT B)

1. Look for `REPLACEMENT POINT B (part 1 of 2)` in `sketch.ino`. If your library header is not `hey_comet_inferencing.h`, change the name **on both lines** (`__has_include` and `#include`).
2. Set `KWS_KEYWORD_LABEL` to the exact label from your EI project.
3. Build. The boot banner must say `KWS backend  : EDGE_IMPULSE`. If it still says `MOCK_KWS`, the header was not found.
4. Tune `KWS_THRESHOLD` (0.80) and `EI_STRIDE_SAMPLES` (inference every ~256 ms) on real recordings.
5. The EI code (part 2 of 2) uses only the standard SDK API: `signal_t`, `run_classifier()`, `ei_impulse_result_t`, and `EI_CLASSIFIER_*`. For lower latency, consider Edge Impulse's continuous-inference example (`run_classifier_continuous`).

A speech model will **not** fire on the simulated tone pattern. Combine EI with the real INMP441, not with `SimulatedAudioSource`. Use `-DFORCE_MOCK_KWS=1` to force the mock back on.

## 9. How to replace simulated audio with an INMP441 (REPLACEMENT POINT A)

Wiring: `SCK → GPIO 12`, `WS → GPIO 11`, `SD → GPIO 10`, `L/R → GND` (left channel), `VDD → 3V3`, `GND → GND`.

1. Implement `INMP441AudioSource::begin()` / `read()` in `sketch.ino` (outline in the comments at `REPLACEMENT POINT A`). Keep `read()` **non-blocking** and return int16 mono at 16 kHz.
2. Set `#define USE_INMP441 1`. No other firmware change is needed.
3. Bring-up tip: keep `MOCK_KWS` and play a 1000 → 1500 → 2000 Hz sequence (200 ms each) from a phone tone generator near the mic. If it triggers, your I2S path, sample rate and gain are right before you add the EI model.
4. Check levels using the `rms`/`peak` fields in the audio packet. Adjust the 32→16-bit shift if audio is too quiet or clipping.
5. Stream real PCM from `onCaptureFrame()` using the chunk format documented in `publishAudio()`. The server already accepts it. Raise `MQTT_BUFFER_BYTES` to ≥ 1100 and use a private broker.

## 10. Known Wokwi limitations

* There is **no INMP441/I2S microphone part**, so audio is synthetic. That is why `SimulatedAudioSource` exists.
* `libraries.txt` resolves Arduino Library Manager names. An Edge Impulse export is a ZIP, not a registry library, and I have not verified loading one into Wokwi. The dependable path for the real model is Arduino IDE/PlatformIO on hardware. Without the header, the sketch falls back to `MOCK_KWS` automatically.
* The simulator may run slower than real time. All timing uses simulated `millis()`/`micros()`, so behaviour is identical, but wall-clock latency in Wokwi is not representative of hardware.
* The public Wokwi gateway reaches internet brokers. A broker on your laptop is not reachable from the browser simulator. Use a public or cloud broker, or Wokwi's private gateway if you have it.
* `PubSubClient::connect()` blocks for up to 5 s (`MQTT_SOCKET_TIMEOUT_S`). The mic "DMA" keeps running and old audio is dropped (`[AUDIO] Buffer overrun` after a reconnect is expected).
* The board's pin names are taken from the Wokwi ESP32-S3-DevKitC-1 part (`esp:4`, `esp:GND.1`, …). If Wokwi reports an unknown pin, hover over the board pin to see its exact name.

## Moving to real hardware (checklist)

* Board: **ESP32S3 Dev Module**. If you use the native USB port for logs, enable **USB CDC On Boot**.
* Copy `secrets.example.h` → `secrets.h` for your Wi-Fi/broker settings, and set `WIFI_CHANNEL 0`.
* Apply replacement points **A** (mic) and **B** (model).
