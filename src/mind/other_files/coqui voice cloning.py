from TTS.api import TTS
import torch
import os
from mind.utils.logging_handler import setup_logger

from TTS.tts.configs.xtts_config import XttsConfig  # Import the problematic class

logger = setup_logger(__name__)

# Whitelist for PyTorch 2.6+ secure loading
torch.serialization.add_safe_globals([XttsConfig])

# Load model on CPU
logger.info("Loading TTS model...")
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to("cpu")
logger.info("TTS model loaded")

# Your 12-sec audio (update path if needed)
audio_path = "/home/badri/mine/hitomi/mind/src/mind/assets/japanese_audio.mp3"  # Or .wav
if not os.path.exists(audio_path):
    raise FileNotFoundError(f"Audio not found: {audio_path}")

# Extract embedding
logger.info("Computing embedding from audio", extra={"audio_path": audio_path})
embedding = tts.synthesizer.speaker_manager.compute_embedding_from_clip(audio_path)
logger.info("Embedding ready", extra={"shape": list(embedding.shape)})

# Save it
output_path = "/home/badri/mine/hitomi/mind/src/mind/assets/my_voice_embedding.pt"
torch.save(embedding, output_path)
logger.info("Saved embedding", extra={"output_path": output_path})