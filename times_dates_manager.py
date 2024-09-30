import time
import threading

class Timer():
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


 	



		
	
		
