# Architecture: SIH 2026 Edge Voice Activator

## 1. End-to-end pipeline

```
 ┌──────────────────────────── ESP32-S3 ────────────────────────────┐
 │                                                                  │
 │  AudioSource ──► 512-sample blocks ──► runKWS() ──► p ≥ 0.80 ?   │
 │  (SIM | INMP441)  (32 ms, "DMA")       (MOCK | EI)      │ yes    │
 │        ▲  REPLACEMENT POINT A              ▲ POINT B    ▼        │
 │        │                                         publish event   │
 │        └──────── always running ─────────        capture 1 s     │
 │                                                  publishAudio()  │
 └──────────────────────────────────────────────────────┬───────────┘
                         Wi-Fi ── MQTT broker ──────────┘  ▲
                                      │                    │ command JSON
                          ┌───────────▼─────────────┐      │
                          │ cloud_server/server.py  │──────┘
                          │  asr.transcribe()       │
                          │  text_to_command()      │
                          └─────────────────────────┘
```

Design principles:

* **Edge first.** Nothing leaves the device until the local KWS fires. Continuous listening costs no bandwidth and no cloud compute, and keeps privacy.
* **One seam per replacement.** Hardware and model swaps touch one class and one function respectively. The state machine, networking and cloud never change.
* **Honest simulation.** Every simulated part names itself in the logs and the payloads (`[MIC-SIM]`, `[MOCK_KWS]`, `"simulated": true`, `"asr_mode": "MOCK_ASR"`).

## 2. What is real vs simulated

| Layer | Wokwi build | Real hardware |
|---|---|---|
| Microphone | `SimulatedAudioSource` (synthetic) | `INMP441AudioSource` over I2S (to implement) |
| Buffering | micros()-paced backlog, 4 × 512 samples, overrun counting | I2S DMA ring (same sizes) |
| KWS | `MOCK_KWS` (Goertzel tone sequence) | Edge Impulse model |
| Wi-Fi | **real** (Wokwi-GUEST) | real |
| MQTT | **real** broker | real (use a private broker) |
| Audio upload | JSON summary | binary PCM chunks + `AUDIO_END` |
| ASR | `MOCK_ASR` | `REAL_ASR` (to implement) |

## 3. Device state machine

| State | Entry action | Leaves when | Next |
|---|---|---|---|
| `BOOT` | banner, GPIO, MQTT client setup | immediately | `WIFI_CONNECTING` |
| `WIFI_CONNECTING` | `connectWiFi()` (non-blocking) | connected / 15 s timeout (retry after 3 s) | `MQTT_CONNECTING` |
| `MQTT_CONNECTING` | `connectMQTT()` with LWT, subscribe `command`, publish `ONLINE` | success / fail (retry after 3 s) / Wi-Fi lost | `LISTENING` |
| `LISTENING` | start mic once, reset KWS | `runKWS()` probability ≥ threshold | `KEYWORD_DETECTED` |
| `KEYWORD_DETECTED` | log | publishes `KEYWORD_DETECTED`, new `request_id` | `CAPTURING` |
| `CAPTURING` | reset capture stats | 1 s of samples captured (abort if stalled for 3 s) | `UPLOADING` |
| `UPLOADING` | none | `publishAudio()` ok / failed | `WAITING_FOR_ASR` / `LISTENING` |
| `WAITING_FOR_ASR` | clear pending | matching command arrives / 10 s timeout | `COMMAND_RECEIVED` / `LISTENING` |
| `COMMAND_RECEIVED` | `handleCloudCommand()` | after 1.2 s | `LISTENING` |

In every state from `LISTENING` onwards, lost Wi-Fi goes to `WIFI_CONNECTING` and lost MQTT goes to `MQTT_CONNECTING`.

All timing uses `millis()`. There are no `delay()` calls. The only blocking call is `PubSubClient::connect()`, bounded to 5 s.

The MQTT callback never changes state. It validates the command and sets a flag, and the state machine consumes the flag in `loop()`.

## 4. Audio front end

* 16 kHz, mono, signed 16-bit, blocks of 512 samples (32 ms). The block size matches a typical `dma_buf_len`, and a backlog of 4 blocks matches `dma_buf_count`.
* The source produces samples on its own clock. If `loop()` stalls, the oldest audio is dropped and an overrun is counted, which is exactly what a real I2S ring does.
* The simulated signal is a pure function of the absolute sample index, so runs are bit-exact reproducible, even across overruns.

## 5. Keyword spotting contract

`KwsResult runKWS(const int16_t* block, size_t n)` returns `{evaluated, probability}`.

* **MOCK_KWS** takes 3 Goertzel filters (1000/1500/2000 Hz, exact bins for N=512 at 16 kHz) on each frame, and labels the frame by the tone holding ≥ 50 % of its energy. The score is the average over the three tones of `min(frames_seen, 5) / 5` for the ordered sequence. The full pattern scores 1.00; the 1000 Hz-only distractor scores 0.33.
* **Edge Impulse** keeps a 1 s ring window and calls `run_classifier()` every 4096 samples. The result is the probability of `KWS_KEYWORD_LABEL`. After a detection the window is cleared, so the same utterance cannot re-trigger.

## 6. MQTT contract

| Topic | Dir | QoS | Retain | Payload |
|---|---|---|---|---|
| `sih/<id>/status` | dev→cloud | 1 (LWT) / 0 | yes | `ONLINE` / `OFFLINE` |
| `sih/<id>/event` | dev→cloud | 0 | no | `KEYWORD_DETECTED` |
| `sih/<id>/audio` | dev→cloud | 0 | no | JSON summary, or binary chunks + `AUDIO_END` JSON |
| `sih/<id>/command` | cloud→dev | 1 | no | `{"request_id","command","text","asr_mode","simulated_audio"}` |

`request_id` = `<DEVICE_ID>-<4-digit seq>`, and the device drops responses with a different id. Commands are never retained, so a reconnecting device does not replay an old command.

## 7. Real PCM streaming design (for the INMP441 build)

* **Where:** `onCaptureFrame()` in `sketch.ino`. Publish each 32 ms block **as it is captured**, so upload overlaps speaking. Do not buffer the whole utterance first.
* **Frame:** `<u32 seq><u32 chunk_index><u32 sample_count>` (LE), then PCM int16 LE. At 512 samples that is 1036 bytes per message, so set `MQTT_BUFFER_BYTES` ≥ 1100.
* **End:** `{"type":"AUDIO_END","request_id":"device01-0007","seq":7}`.
* **Bandwidth:** 16 000 × 2 B = 32 kB/s ≈ 256 kbit/s. That is fine on local Wi-Fi. If needed, reduce it with IMA-ADPCM (4:1) or Opus.
* **Termination:** a fixed 1–2 s capture is simplest. A VAD-based end-of-speech cut is lower latency and a good next step.
* **Broker:** use a private broker (Mosquitto on the ASR laptop or a cloud broker) with authentication and TLS where possible. Never stream voice through a public test broker.
* `cloud_server/server.py` already implements the receive side of this protocol.

## 8. Latency budget (targets to measure on hardware)

| Stage | Budget driver |
|---|---|
| Wake-word decision | KWS window + stride (EI: ≤ 256 ms stride; continuous mode is lower) |
| Event publish | persistent MQTT connection, no connect on the hot path |
| Capture | overlaps with streaming, so it is not additive when streaming per block |
| ASR | engine and model size on the laptop |
| Command return | one MQTT round trip |

Measure with `uptime_ms` in the audio packet and timestamps in the server log. Wokwi timings are not representative.

## 9. Security notes

* No credentials in source. The device takes overrides from `secrets.h` (git-ignored), and the server reads them from env or `.env` (git-ignored).
* The default public broker is for synthetic demos only.
* The device validates command JSON, state and `request_id`, and ignores everything else. Only a fixed command set has effects.
