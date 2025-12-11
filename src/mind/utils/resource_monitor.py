# mind/utils/resource_monitor.py
import os
import time
import psutil
import threading
import logging
from datetime import datetime
from mind.utils import setup_logger


logger = setup_logger(__name__, log_file="resource.log" ,level=logging.INFO)

class ResourceMonitor:
    def __init__(self, interval=2.0):
        self.interval = interval
        self.process = psutil.Process(os.getpid()) # Monitor current process
        self.running = False
        self.thread = None
        
        # Stats for averaging
        self.cpu_samples = []
        self.mem_samples = [] # In MB
        
    def _monitor_loop(self):
        # Initial blocking call to prime CPU counter
        self.process.cpu_percent(interval=None)
        
        start_mem = self.process.memory_info().rss / 1024 / 1024
        logger.info(f"--- MONITOR STARTING --- | Initial RAM: {start_mem:.2f} MB")

        while self.running:
            try:
                # Get metrics
                cpu = self.process.cpu_percent(interval=None)
                mem = self.process.memory_info().rss / 1024 / 1024 # Convert Bytes to MB
                
                self.cpu_samples.append(cpu)
                self.mem_samples.append(mem)

                # Log current state
                logger.info(f"CPU: {cpu:.1f}% | RAM: {mem:.2f} MB")
                
                time.sleep(self.interval)
            except Exception as e:
                logger.error(f"Monitor error: {e}")
                break

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        
        self._print_summary()

    def _print_summary(self):
        if not self.cpu_samples:
            return

        avg_cpu = sum(self.cpu_samples) / len(self.cpu_samples)
        max_cpu = max(self.cpu_samples)
        avg_mem = sum(self.mem_samples) / len(self.mem_samples)
        max_mem = max(self.mem_samples)

        summary = (
            f"\n=== RESOURCE SUMMARY ===\n"
            f"Duration: {len(self.cpu_samples) * self.interval} seconds\n"
            f"Avg CPU:  {avg_cpu:.2f}%\n"
            f"Max CPU:  {max_cpu:.2f}%\n"
            f"Avg RAM:  {avg_mem:.2f} MB\n"
            f"Max RAM:  {max_mem:.2f} MB\n"
            f"========================"
        )
        logger.info(summary)