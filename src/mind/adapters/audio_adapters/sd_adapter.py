#TODO:not working with wav files
from mind.core.ports.percieve_port import Audition
from mind.core.ports.act_port import Speaker
from mind.utils.logging_handler import setup_logger
import sounddevice as sd


logger = setup_logger(__name__)

class AudioManager(Audition, Speaker):
    """
    Handles Audio Input (Mic) and Output (Speakers) using sounddevice.
    Works natively with NumPy arrays.
    """
    def __init__(self, rate=44100, channels=2, dtype='int16'):
        self.rate = rate
        self.channels = channels
        self.dtype = dtype
        
        # Streams
        self.output_stream = None
        self.input_stream = None

    # --- OUTPUT (Playback) Methods ---
    def start_output_stream(self, rate=None, channels=None):
        """Initialize the speaker output stream."""
        # Update config if provided (e.g., from video file metadata)
        if rate: self.rate = rate
        if channels: self.channels = channels

        if self.output_stream is None:
            self.output_stream = sd.OutputStream(
                samplerate=self.rate,
                channels=self.channels,
                dtype=self.dtype,
                latency='high'  # FIX: Increases buffer to prevent underruns (try 'low' for less delay)
            )
            self.output_stream.start()

    def write_chunk(self, data_array):
        """
        Write numpy array to speakers.
        data_array shape should be (frames, channels).
        Blocks if buffer is full (syncing audio automatically).
        """
        if self.output_stream:
            try:
                self.output_stream.write(data_array)
            except Exception as e:
                logger.error("Audio write error", exc_info=e)

    def say(self, data_array):
        """Alias for write_chunk to comply with Speaker interface."""
        self.write_chunk(data_array)

    # --- INPUT (Microphone) Methods ---
    def start_input_stream(self, device_index=None):
        """Initialize the microphone input stream."""
        if self.input_stream is None:
            self.input_stream = sd.InputStream(
                samplerate=self.rate,
                channels=self.channels,
                dtype=self.dtype,
                device=device_index
            )
            self.input_stream.start()

    def read_chunk(self, num_frames=1024):
        """
        Read 'num_frames' from microphone.
        Returns: numpy array of shape (num_frames, channels)
        """
        if self.input_stream:
            try:
                data, overflow = self.input_stream.read(num_frames)
                if overflow:
                    logger.warning("Audio input overflow")
                return data
            except Exception as e:
                logger.error("Audio read error", exc_info=e)
                return None
        return None
    
    def listen(self, num_frames=1024):
        """Alias for read_chunk to comply with Audition interface."""
        return self.read_chunk(num_frames=num_frames)

    def close(self):
        """Cleanup streams."""
        if self.output_stream:
            self.output_stream.stop()
            self.output_stream.close()
            self.output_stream = None
            
        if self.input_stream:
            self.input_stream.stop()
            self.input_stream.close()
            self.input_stream = None