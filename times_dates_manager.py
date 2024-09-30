import time
import threading


class Timer:
	def __init__(self, duration):
		self.duration = duration
		self.remaining = duration
		self.control_thread = threading.Event()
		self.control_thread.set()

	def start(self):
		threading.Thread(target = self.run).start()
				
	def run(self):
		print("  ",self.remaining)
		while self.control_thread.is_set() and self.remaining > 0:
			time.sleep(1)
			self.remaining -= 1
			print("   ",self.remaining)

	def pause(self):
		self.control_thread.clear()
		
	def resume(self):
		self.control_thread.set()
		
	def reset(self, duration):
		self.pause()
		self.__init__(duration)
		threading.Thread(target = self.run).start()


class Stopwatch:
	def __init__(self):
		self.marks = {}
		self.running = False
		self.started_at = None

	def start(self):
		self.running = True
		self.started_at = time.perf_counter()

	def mark(self, name='unknown'):
		if self.running:
			self.marks[name] = time.perf_counter() - self.started_at
		else:
			return None # handle or raise exception

	def stop(self):
		self.running = False
		return time.perf_counter - self.started_at


	
		
