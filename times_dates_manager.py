import time
import threading
# from tabulate import tabulate


class Timer:
    def __init__(self):
        self.control_event = threading.Event()
        self.control_event.set()
        self.completed = False
        self.new = True


    def start(self, sec, mins=0, hrs=0, name = None):
        def inner_start():
            self.duration = (hrs * 3600) + (mins * 60) + sec
            self.remaining = self.duration
            self.thread = threading.Thread(target=self.run, name = name )
            self.completed = False
            self.thread.start()
        if self.new:
            inner_start()
            self.new = False
        else:
            if self.completed:
                inner_start()
            else:
                print(f'The thread {self.thread.name} with id - {self.thread.ident} is still running....\
                \nSo you can not start a new timer with same the timer object')

        # what if same timer object starts a new thread before completion.

    def run(self):
        print("  ", self.remaining)
        while self.remaining > 0:
            self.control_event.wait()
            time.sleep(1)
            self.remaining -= 1
            print("   ", self.remaining)
        self.completed = True

    def pause(self):
        self.control_event.clear()

    def resume(self):
        self.control_event.set()

    def reset(self, duration):  # check whether this works or not.
        self.pause()
        self.remaining = self.duration
        self.control_event.set()

    def make_sound(self):
        pass  # play some audio.



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
            return None  # handle or raise exception

    def stop(self):
        self.running = False
        for i,j in self.marks.items():
            print(f"{i:<6} --- {j}")
        # print(tabulate(self.marks.items()))
        # print("stopped sw")
        return int(time.monotonic() - self.started_at)


class Alarm:

    def __init__(self, hrs, mins, sec=0):
        self.timer = Timer()
        self.timer.start(hrs=hrs, mins=mins, sec=sec, name = 'alarm')
        # you can implement better approach of letting tgread to go to sleep for some time.


class Pomodoro:
    timings = ((10, 5, 7),(1500, 300, 600), (3000, 600, 1200))

    def __init__(self, general=True):
        if general:
            self.work_sec, self.break_, self.lbreak = Pomodoro.timings[0]
        else:
            self.work_sec, self.break_, self.lbreak = Pomodoro.timings[1]
        self.timer = Timer()
        self.counter = 0
        self.new = True

    def start(self):
        allow = True
        def run():
            print('work..........')
            self.timer.start(self.work_sec, name='pomodoro-w')
            self.counter = (self.counter % 4) + 1

        if self.new:
            run()
            self.new = False
        else:
            if not self.timer.thread.is_alive():
                run()
            elif self.timer.thread.is_alive():
                print(f'The thread {self.timer.thread.name} with id - {self.timer.thread.ident} is still running....\
                \nSo you can not start a new timer with same the timer object')
                # check this condition also. I mean trying to start pomodoro when previous thread is still running.
        while allow:
            if self.timer.completed:
                print('starting break..')
                if self.counter < 4:
                    self.take_break(self.break_)
                else:
                    self.take_break(self.lbreak)
                allow = False
            time.sleep(0.1)


    def take_break(self, sec):
        print("break.....")
        self.timer.start(sec, name= 'pomodoro-b')

    def pause(self):
        self.timer.pause()

    def resume(self):
        self.timer.resume()

    def reset(self):
        self.timer.remaining = 0
        self.counter = 0
        self.start()


