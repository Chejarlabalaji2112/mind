import time
from tools.time_tools.timer import Timer
from tools.time_tools.stopwatch import Stopwatch
from tools.time_tools.pomodoro import Pomodoro, PomodoroPhase
from tools.time_tools.clock import Clock

from utils.time_conversions import convert_to_seconds, format_seconds_to_hms

def timer_tick_handler(remaining_time, remaining_time_formatted):
    print(f"Timer Tick: {remaining_time_formatted} remaining ({remaining_time:.1f}s)")

def timer_finished_handler():
    print("Timer: Time's up!")

def stopwatch_tick_handler(elapsed_time, elapsed_time_formatted):
    print(f"Stopwatch Tick: {elapsed_time_formatted} elapsed ({elapsed_time:.1f}s)")

def stopwatch_lap_handler(lap_time, lap_time_formatted, all_laps):
    print(f"Stopwatch Lap: {lap_time_formatted} ({lap_time:.1f}s), All Laps: {[format_seconds_to_hms(l) for l in all_laps]}")

def pomodoro_tick_handler(remaining_time, phase, remaining_time_formatted):
    print(f"Pomodoro ({phase}) Tick: {remaining_time_formatted} remaining ({remaining_time:.1f}s)")

def pomodoro_work_start_handler(cycle):
    print(f"Pomodoro: Starting Work Phase (Cycle {cycle})")

def pomodoro_break_start_handler(cycle):
    print(f"Pomodoro: Starting Break Phase (Cycle {cycle})")

def pomodoro_long_break_start_handler(cycle):
    print(f"Pomodoro: Starting Long Break Phase (Cycle {cycle})")

def pomodoro_phase_end_handler(previous_phase, current_cycle):
    print(f"Pomodoro: {previous_phase} phase ended (Cycle {current_cycle})")

def pomodoro_cycle_complete_handler(cycle):
    print(f"Pomodoro: Cycle {cycle} completed!")


def demonstrate_timer():
    print("\n--- Demonstrating Timer ---")
    timer = Timer()
    timer.on_tick.add_listener(timer_tick_handler)
    timer.on_finished.add_listener(timer_finished_handler)

    timer_duration = convert_to_seconds(minutes=0, seconds=5) # 5-second timer
    timer.start(timer_duration)
    time.sleep(2)
    timer.pause()
    print(f"Timer status after pause: {timer.get_status()['remaining_time_formatted']}")
    time.sleep(1)
    timer.resume()
    time.sleep(3) # Let it finish
    print(f"Timer status after finish: {timer.get_status()['remaining_time_formatted']}")
    timer.reset()
    print(f"Timer status after reset: {timer.get_status()['remaining_time_formatted']}")

def demonstrate_stopwatch():
    print("\n--- Demonstrating Stopwatch ---")
    stopwatch = Stopwatch()
    stopwatch.on_tick.add_listener(stopwatch_tick_handler)
    stopwatch.on_lap.add_listener(stopwatch_lap_handler)

    stopwatch.start()
    time.sleep(1.5)
    stopwatch.lap()
    time.sleep(1)
    stopwatch.pause()
    print(f"Stopwatch status after pause: {stopwatch.get_status()['elapsed_time_formatted']}")
    time.sleep(1)
    stopwatch.resume()
    time.sleep(0.8)
    stopwatch.lap()
    time.sleep(1.2)
    stopwatch.stop()
    print(f"Stopwatch status after stop: {stopwatch.get_status()['elapsed_time_formatted']}")
    stopwatch.reset()
    print(f"Stopwatch status after reset: {stopwatch.get_status()['elapsed_time_formatted']}")

def demonstrate_pomodoro():
    print("\n--- Demonstrating Pomodoro (short durations for demo) ---")
    # Using short durations for demonstration purposes
    pomodoro = Pomodoro(work_duration_s=5, short_break_duration_s=3, long_break_duration_s=7, cycles_before_long_break=2)
    pomodoro.on_tick.add_listener(pomodoro_tick_handler)
    pomodoro.on_work_start.add_listener(pomodoro_work_start_handler)
    pomodoro.on_short_break_start.add_listener(pomodoro_break_start_handler)
    pomodoro.on_long_break_start.add_listener(pomodoro_long_break_start_handler)
    pomodoro.on_phase_end.add_listener(pomodoro_phase_end_handler)
    pomodoro.on_cycle_complete.add_listener(pomodoro_cycle_complete_handler)

    pomodoro.start()
    # Let it run through a few phases
    time.sleep(6) # Work phase (5s) + a bit
    print(f"Pomodoro status after work phase: {pomodoro.get_status()['remaining_time_formatted']}")
    time.sleep(4) # Short break (3s) + a bit
    print(f"Pomodoro status after short break: {pomodoro.get_status()['remaining_time_formatted']}")
    time.sleep(6) # Work phase (5s) + a bit
    print(f"Pomodoro status after second work phase: {pomodoro.get_status()['remaining_time_formatted']}")
    time.sleep(8) # Long break (7s) + a bit
    print(f"Pomodoro status after long break: {pomodoro.get_status()['remaining_time_formatted']}")
    pomodoro.stop()
    print(f"Pomodoro status after stop: {pomodoro.get_status()['remaining_time_formatted']}")
    pomodoro.reset()
    print(f"Pomodoro status after reset: {pomodoro.get_status()['remaining_time_formatted']}")

def demonstrate_clock():
    print("\n--- Demonstrating Clock ---")
    print(f"Current UTC Datetime: {Clock.get_current_utc_datetime()}")
    print(f"Current Local Datetime: {Clock.get_current_local_datetime()}")
    print(f"Current Datetime (New York): {Clock.get_current_datetime(timezone_str='America/New_York')}")
    print(f"Current Time (London): {Clock.get_current_time_str(timezone_str='Europe/London')}")
    print(f"Current Date (Tokyo): {Clock.get_current_date_str(timezone_str='Asia/Tokyo')}")
    print(f"Current Datetime (India): {Clock.get_current_datetime_str(timezone_str='Asia/Kolkata')}")
    print(f"Current Datetime (Invalid TZ): {Clock.get_current_datetime_str(timezone_str='Invalid/Timezone')}")


if __name__ == "__main__":
    demonstrate_timer()
    demonstrate_stopwatch()
    demonstrate_pomodoro()
    demonstrate_clock()
