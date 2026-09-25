"""
ASR (speech-to-text) backends for the SIH 2026 cloud server.

Public interface (what server.py uses):

    transcribe(audio_data, sample_rate=16000) -> ASRResult

`audio_data` is either
  * dict  - the SIMULATED_AUDIO_SUMMARY JSON sent by the Wokwi build
            (no real audio inside), or
  * bytes - raw PCM, signed 16-bit little-endian, mono, `sample_rate` Hz
            (what a real INMP441 device will stream).

Backends
  MOCK_ASR  (default) deterministic fake transcripts; works everywhere.
  REAL_ASR  PLACEHOLDER. Selected with ASR_MODE=real. It checks that an ASR
            engine package is importable, but the actual engine call is left
            for you to write (see RealASR.transcribe). Until then, and whenever
            the input is not real PCM, the service falls back to MOCK_ASR so
            the server never stops.

Environment variables
  ASR_MODE           mock | real            (default: mock)
  ASR_ENGINE_MODULE  python module to probe  (default: whisper)
                     e.g. whisper, faster_whisper, vosk, pywhispercpp
"""

from __future__ import annotations

import importlib.util
import logging
import os
from dataclasses import dataclass
from typing import Optional, Union

log = logging.getLogger("asr")

AudioInput = Union[bytes, bytearray, dict]


@dataclass
class ASRResult:
    text: str
    mode: str                       # "MOCK_ASR" or "REAL_ASR"
    confidence: Optional[float] = None
    note: str = ""


# ---------------------------------------------------------------------------
# MOCK_ASR
# ---------------------------------------------------------------------------
class MockASR:
    """Deterministic fake transcripts. NOT speech recognition.

    The phrase is picked from the device's request sequence number, so a
    given run always produces the same command sequence:
    seq 1 -> "turn on the light", 2 -> "turn off the light", 3 -> status, ...
    """

    mode = "MOCK_ASR"
    PHRASES = ("turn on the light", "turn off the light", "what is the status")

    def __init__(self) -> None:
        self._counter = 0

    def transcribe(self, audio_data: AudioInput, sample_rate: int = 16000) -> ASRResult:
        seq = audio_data.get("seq") if isinstance(audio_data, dict) else None
        if not isinstance(seq, int) or seq < 1:
            self._counter += 1
            seq = self._counter
        text = self.PHRASES[(seq - 1) % len(self.PHRASES)]
        if isinstance(audio_data, (bytes, bytearray)):
            note = f"mock transcript for {len(audio_data) // 2} PCM samples"
        else:
            note = "mock transcript for simulated audio summary"
        return ASRResult(text=text, mode=self.mode, confidence=None, note=note)


# ---------------------------------------------------------------------------
# REAL_ASR (placeholder)
# ---------------------------------------------------------------------------
class RealASR:
    """Placeholder for a real engine (Whisper, whisper.cpp bindings, Vosk...).

    Nothing here calls an ASR library yet. Implement `transcribe()` for the
    engine you choose, following that engine's own documentation.
    """

    mode = "REAL_ASR"

    def __init__(self, engine_module: str) -> None:
        self.engine_module = engine_module
        # >>> Load your model ONCE here (loading is slow; do not do it per request).

    def transcribe(self, audio_data: AudioInput, sample_rate: int = 16000) -> ASRResult:
        if not isinstance(audio_data, (bytes, bytearray)):
            # The Wokwi build only sends a JSON summary: there is nothing to recognise.
            raise ValueError("REAL_ASR needs raw PCM bytes; received a simulated audio summary")
        # >>> INSERT REAL ASR HERE
        #     1. Convert `audio_data` (int16 LE mono @ sample_rate) into whatever
        #        your engine expects (many Python engines want float32 in [-1, 1]:
        #        numpy.frombuffer(audio_data, dtype="<i2").astype("float32") / 32768).
        #     2. Run the engine, get the transcript text.
        #     3. return ASRResult(text=..., mode=self.mode, confidence=...)
        raise NotImplementedError(
            f"REAL_ASR placeholder: implement RealASR.transcribe() for '{self.engine_module}'"
        )


def engine_available(module_name: str) -> bool:
    try:
        return importlib.util.find_spec(module_name) is not None
    except (ImportError, ValueError):
        return False


# ---------------------------------------------------------------------------
# Service with automatic fallback
# ---------------------------------------------------------------------------
class ASRService:
    def __init__(self, mode: Optional[str] = None, engine_module: Optional[str] = None) -> None:
        mode = (mode or os.getenv("ASR_MODE", "mock")).strip().lower()
        engine_module = engine_module or os.getenv("ASR_ENGINE_MODULE", "whisper")
        self.mock = MockASR()
        self.real: Optional[RealASR] = None
        self._warned: set[str] = set()

        if mode == "real":
            if engine_available(engine_module):
                self.real = RealASR(engine_module)
                log.info("ASR: REAL_ASR selected (engine module '%s' found; placeholder until implemented)",
                         engine_module)
            else:
                log.warning("ASR: ASR_MODE=real but module '%s' is not installed -> running in MOCK_ASR mode",
                            engine_module)
        elif mode != "mock":
            log.warning("ASR: unknown ASR_MODE=%r -> running in MOCK_ASR mode", mode)

        if self.real is None:
            log.info("ASR: MOCK_ASR mode (deterministic fake transcripts, no speech recognition)")

    @property
    def active_mode(self) -> str:
        return self.real.mode if self.real else self.mock.mode

    def transcribe(self, audio_data: AudioInput, sample_rate: int = 16000) -> ASRResult:
        if self.real is not None:
            try:
                return self.real.transcribe(audio_data, sample_rate)
            except (NotImplementedError, ValueError) as exc:
                if str(exc) not in self._warned:        # warn once per reason
                    self._warned.add(str(exc))
                    log.warning("ASR: %s -> falling back to MOCK_ASR", exc)
        return self.mock.transcribe(audio_data, sample_rate)


_service: Optional[ASRService] = None


def get_service() -> ASRService:
    global _service
    if _service is None:
        _service = ASRService()
    return _service


def transcribe(audio_data: AudioInput, sample_rate: int = 16000) -> ASRResult:
    """Module-level interface: transcribe(audio_data) -> ASRResult."""
    return get_service().transcribe(audio_data, sample_rate)
