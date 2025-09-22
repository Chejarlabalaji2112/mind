import sys
import os

# Add the parent directory of 'mind' to the Python path
# This is necessary to import modules from 'mind' correctly when running this script directly
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from mind.adapters.system_speaker_adapter import SystemSpeakerAdapter

def test_speaker():
    print("Initializing SystemSpeakerAdapter...")
    speaker = SystemSpeakerAdapter()
    print("Adapter initialized. Attempting to say something...")
    success = speaker.say("Hello, this is a test from the system speaker adapter.")
    if success:
        print("Speaker test successful!")
    else:
        print("Speaker test failed.")

if __name__ == "__main__":
    test_speaker()
