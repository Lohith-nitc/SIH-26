#!/usr/bin/env python3
"""
SIH 2026 cloud server — MQTT bridge between the ESP32-S3 and ASR.

  subscribes  sih/<device>/event    "KEYWORD_DETECTED"
              sih/<device>/audio    JSON audio summary (Wokwi build)
                                    or binary PCM chunks + AUDIO_END (real device)
              sih/<device>/status   retained ONLINE / OFFLINE (for logging)
  publishes   sih/<device>/command  {"request_id", "command", "text", "asr_mode", ...}

Configuration: environment variables, optionally loaded from cloud_server/.env
(see .env.example). No credentials are stored in this file.

Run:  python server.py
"""

from __future__ import annotations

import json
import logging
import os
import signal
import struct
import sys
import time
import uuid
from pathlib import Path

import paho.mqtt.client as mqtt

import asr

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def load_dotenv(path: Path) -> None:
    """Minimal .env loader (KEY=VALUE lines). Real env vars always win."""
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


load_dotenv(Path(__file__).with_name(".env"))

MQTT_BROKER = os.getenv("MQTT_BROKER", "test.mosquitto.org")   # public test broker, no auth
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME") or None
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD") or None
MQTT_TLS = os.getenv("MQTT_TLS", "0").lower() in ("1", "true", "yes")
DEVICE_ID = os.getenv("DEVICE_ID", "device01")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID") or f"sih-cloud-{DEVICE_ID}-{uuid.uuid4().hex[:6]}"
SAMPLE_RATE = 16000

TOPIC_STATUS = f"sih/{DEVICE_ID}/status"
TOPIC_EVENT = f"sih/{DEVICE_ID}/event"
TOPIC_AUDIO = f"sih/{DEVICE_ID}/audio"
TOPIC_COMMAND = f"sih/{DEVICE_ID}/command"

# Binary PCM chunk header for the real-device stream (see docs/architecture.md):
#   <u32 request_seq><u32 chunk_index><u32 sample_count>  little-endian, then PCM int16 LE
PCM_HEADER = struct.Struct("<III")
MAX_PCM_BYTES = SAMPLE_RATE * 2 * 10          # refuse > 10 s per request

logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO"),
    format="%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("server")

# ---------------------------------------------------------------------------
# Command processing
# ---------------------------------------------------------------------------

def text_to_command(text: str) -> str:
    """Very small intent parser: transcript -> device command."""
    t = text.lower()
    if "turn on" in t or "switch on" in t:
        return "TURN_ON"
    if "turn off" in t or "switch off" in t:
        return "TURN_OFF"
    if "status" in t:
        return "STATUS"
    return "UNKNOWN"


class CloudServer:
    def __init__(self) -> None:
        self.asr = asr.get_service()
        self.pcm_buffers: dict[int, dict[int, bytes]] = {}   # seq -> {chunk_index: pcm}
        self.stats = {"events": 0, "audio": 0, "commands": 0}

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=MQTT_CLIENT_ID)
        if MQTT_USERNAME:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        if MQTT_TLS:
            self.client.tls_set()
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

    # ---- MQTT callbacks ---------------------------------------------------
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        log.info("MQTT connected to %s:%d as %s", MQTT_BROKER, MQTT_PORT, MQTT_CLIENT_ID)
        for topic in (TOPIC_EVENT, TOPIC_AUDIO, TOPIC_STATUS):
            client.subscribe(topic, qos=1)
            log.info("Subscribed %s", topic)
        log.info("ASR mode: %s. Waiting for the device...", self.asr.active_mode)

    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            log.warning("MQTT disconnected (%s) - paho will reconnect", reason_code)

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == TOPIC_EVENT:
                self.handle_event(msg.payload)
            elif msg.topic == TOPIC_AUDIO:
                self.handle_audio(msg.payload)
            elif msg.topic == TOPIC_STATUS:
                log.info("[STATUS] device %s is %s%s", DEVICE_ID,
                         msg.payload.decode(errors="replace"), " (retained)" if msg.retain else "")
        except Exception:                         # never let one bad message kill the server
            log.exception("Error handling message on %s", msg.topic)

    # ---- handlers -----------------------------------------------------------
    def handle_event(self, payload: bytes) -> None:
        text = payload.decode(errors="replace").strip()
        self.stats["events"] += 1
        if text == "KEYWORD_DETECTED":
            log.info("[EVENT] KEYWORD_DETECTED - device is capturing the command")
        else:
            log.info("[EVENT] %s", text)

    def handle_audio(self, payload: bytes) -> None:
        # A structurally valid binary chunk (12-byte header + exactly
        # sample_count*2 bytes of PCM) is PCM; everything else must be JSON.
        # Checking the binary shape first avoids misreading a chunk whose
        # first byte happens to be '{' (request seq 123).
        if len(payload) >= PCM_HEADER.size:
            seq, index, count = PCM_HEADER.unpack_from(payload)
            if len(payload) - PCM_HEADER.size == count * 2 and count > 0:
                self.handle_pcm_chunk(seq, index, count, payload[PCM_HEADER.size:])
                return
        try:
            doc = json.loads(payload)
        except (UnicodeDecodeError, json.JSONDecodeError):
            log.warning("[AUDIO] ignoring %d-byte message: neither JSON nor a valid PCM chunk", len(payload))
            return
        if not isinstance(doc, dict):
            log.warning("[AUDIO] ignoring JSON that is not an object")
            return
        kind = doc.get("type")
        if kind in ("SIMULATED_AUDIO_SUMMARY", "AUDIO_SUMMARY"):
            self.stats["audio"] += 1
            log.info("[AUDIO] %s request=%s samples=%s (%s ms) rms=%s peak=%s kws=%s p=%s simulated=%s",
                     kind, doc.get("request_id"), doc.get("samples"), doc.get("duration_ms"),
                     doc.get("rms"), doc.get("peak"), doc.get("kws_backend"),
                     doc.get("kws_probability"), doc.get("simulated"))
            self.respond(doc.get("request_id", ""), self.asr.transcribe(doc, SAMPLE_RATE),
                         simulated=bool(doc.get("simulated", True)))
        elif kind == "AUDIO_END":
            seq = int(doc.get("seq", 0))
            chunks = self.pcm_buffers.pop(seq, {})
            pcm = b"".join(chunks[i] for i in sorted(chunks))
            self.stats["audio"] += 1
            log.info("[AUDIO] AUDIO_END request=%s: %d chunks, %d samples (%.2f s)",
                     doc.get("request_id"), len(chunks), len(pcm) // 2, len(pcm) / 2 / SAMPLE_RATE)
            self.respond(doc.get("request_id", ""), self.asr.transcribe(pcm, SAMPLE_RATE),
                         simulated=False)
        else:
            log.warning("[AUDIO] unknown JSON audio message type %r", kind)

    def handle_pcm_chunk(self, seq: int, index: int, count: int, pcm: bytes) -> None:
        buf = self.pcm_buffers.setdefault(seq, {})
        if sum(map(len, buf.values())) + len(pcm) > MAX_PCM_BYTES:
            log.warning("[AUDIO] request seq %d exceeds %d bytes - dropping chunk", seq, MAX_PCM_BYTES)
            return
        buf[index] = pcm
        log.debug("[AUDIO] seq %d chunk %d: %d samples", seq, index, count)

    def respond(self, request_id: str, result: asr.ASRResult, simulated: bool) -> None:
        command = text_to_command(result.text)
        log.info("[ASR] %s transcript: %r -> command %s", result.mode, result.text, command)
        response = {
            "request_id": request_id,
            "command": command,
            "text": result.text,
            "asr_mode": result.mode,
            "simulated_audio": simulated,
        }
        body = json.dumps(response, separators=(",", ":"))
        info = self.client.publish(TOPIC_COMMAND, body, qos=1, retain=False)
        self.stats["commands"] += 1
        log.info("[COMMAND] -> %s %s (mid=%s)", TOPIC_COMMAND, body, info.mid)

    # ---- lifecycle ------------------------------------------------------------
    def run(self) -> None:
        log.info("Connecting to %s:%d (tls=%s, auth=%s)...", MQTT_BROKER, MQTT_PORT, MQTT_TLS,
                 "yes" if MQTT_USERNAME else "no")
        while True:
            try:
                self.client.connect(MQTT_BROKER, MQTT_PORT, keepalive=30)
                break
            except OSError as exc:
                log.error("Cannot reach broker (%s); retrying in 5 s", exc)
                time.sleep(5)
        self.client.loop_forever(retry_first_connection=True)

    def stop(self, *_):
        log.info("Stopping. Stats: %s", self.stats)
        self.client.disconnect()


def main() -> int:
    server = CloudServer()
    signal.signal(signal.SIGINT, server.stop)
    signal.signal(signal.SIGTERM, server.stop)
    server.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
