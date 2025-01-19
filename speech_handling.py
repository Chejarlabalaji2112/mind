import pyaudio
from vosk import Model, KaldiRecognizer
import json

#model_path = "/media/badri/B/PersonalProject/speech_recog_models/vosk-model-en-en-us-0.15"
model_path = "/home/badri/mine/ser/models_vosk/vosk-model-en"
model = Model(model_path)

rec = KaldiRecognizer(model, 16000)

# Initialize PyAudio
p = pyaudio.PyAudio()

stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
stream.start_stream()


print("Listening...")
while True:
    data = stream.read(4000, exception_on_overflow=False)
    if rec.AcceptWaveform(data):
        result = json.loads(rec.Result())
        print(result['text'])

