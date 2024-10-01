import time
import threading
from tabulate import tabulate

class Timer:
	def __init__(self, sec, mins=0, hrs=0):
		self.duration = (hrs * 3600) + (mins * 60) + sec
		self.remaining = self.duration
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
		
	def make_sound(self):
		pass #play some audio.


class Stopwatch:
	def __init__(self):
		self.marks = {}
		self.running = False
		self.started_at = None

	def start(self):
		print("STARTED SW")
		self.running = True
		self.started_at = time.monotonic()

	def mark(self, name='unknown'):
		print("marked........")
		if self.running:
			self.marks[name] = int(time.monotonic() - self.started_at)
		else:
			return None # handle or raise exception

	def stop(self):
		self.running = False
		'''for i,j in self.marks.items():
			print(f"{i:<6} --- {j}")'''
		print(tabulate(self.marks.items()))
		print("stopped sw")
		return int(time.monotonic() - self.started_at)

		

class Alarm:
		def __init__(self, hrs, mins, sec = 0):
			self.timer = Timer(sec, mins, hrs)
			self.timer.start()
			# you can implement better approach of letting tgread to go to sleep for some time.
			
			
			
class Pomodoro:
			timings = ((1500, 300, 600),(3000, 600, 1200))
			def __init__(general = True):
				if general:
					self.work_sec, break_, lbreak = timings[0]
				else:
				  self.work_sec, break_, lbreak = timings[1]
				self.timer