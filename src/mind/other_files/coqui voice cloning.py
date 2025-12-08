from TTS.api import TTS
import torch
import os

from TTS.tts.configs.xtts_config import XttsConfig  # Import the problematic class

# Whitelist for PyTorch 2.6+ secure loading
torch.serialization.add_safe_globals([XttsConfig])

# Load model on CPU
print("Loading TTS model...")
tts = TTS("tts_models/multilingual/multi-dataset/xtts_v2").to("cpu")
print("Model loaded.")

# Your 12-sec audio (update path if needed)
audio_path = "/home/badri/mine/hitomi/mind/src/mind/assets/japanese_audio.mp3"  # Or .wav
if not os.path.exists(audio_path):
    raise FileNotFoundError(f"Audio not found: {audio_path}")

# Extract embedding
print("Computing embedding from audio...")
embedding = tts.synthesizer.speaker_manager.compute_embedding_from_clip(audio_path)
print(f"Embedding ready (shape: {embedding.shape})")

# Save it
output_path = "/home/badri/mine/hitomi/mind/src/mind/assets/my_voice_embedding.pt"
torch.save(embedding, output_path)
print(f"Saved to: {output_path}")