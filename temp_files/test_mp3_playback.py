import sys
import os

# Add the parent directory of 'mind' to the Python path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from mind.adapters.system_speaker_adapter import SystemSpeakerAdapter

def test_mp3_playback(mp3_file_path: str):
    print(f"Attempting to play MP3 file: {mp3_file_path}")
    speaker = SystemSpeakerAdapter()
    success = speaker.play_audio(mp3_file_path)
    if success:
        print("MP3 playback successful!")
    else:
        print("MP3 playback failed. Check if ffmpeg is installed and the file path is correct.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        mp3_path = sys.argv[1]
        test_mp3_playback(mp3_path)
    else:
        print("Usage: python test_mp3_playback.py <path_to_mp3_file>")
        print("Please provide the path to an MP3 file as a command-line argument.")
