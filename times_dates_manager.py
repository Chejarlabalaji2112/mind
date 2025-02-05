import sqlite3
import time
import threading
import json
from pygame import mixer
mixer.init()

# TODO PERFORM UNIT TEST VERY BADLY
# TODO AND ALSO EXCEPTION HANDLING SHOULD BE DONE
# once timer is started eventhough I interrupt program in pycharm thread is running
# so when we want to interrupt program try like calling all available threads pause method .

# from tabulate import tabulate


  # play some audio


class Timer:
    def __init__(self, silent = True, volume = 1):
        self.control_event = threading.Event()
        self.control_event.set()
        self.completed = False
        self.new = True
        self.silent = silent
        self.sounds = [mixer.Sound("sounds/ticktock.mp3"), mixer.Sound("sounds/alarm.WAV"), mixer.Sound("sounds/complete.oga")]
        self.sounds[0].set_volume(volume)
    def start(self, sec, mins=0, hrs=0, name=None):
        def inner_start():
            self.duration = (hrs * 3600) + (mins * 60) + sec
            self.remaining = self.duration
            self.thread = threading.Thread(target=self.run, name=name)
            self.completed = False
            self.thread.start()
            self.make_sound(0)

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
        self.sounds[0].stop()
        self.sounds[2].play()
        time.sleep(1)
        self.completed = True

    def pause(self):
        self.control_event.clear()
        self.sounds[0].stop()

    def resume(self):
        self.control_event.set()
        time.sleep(1)
        self.make_sound(0)

    def kill(self):
        self.pause()
        self.remaining =0
        self.resume()

    def reset(self):  # check whether this works or not.
        self.pause()
        self.remaining = self.duration
        self.control_event.set()
        time.sleep(1)
        self.make_sound(0)

    def make_sound(self, i): # here `i` determines which sound to activate.
        if self.silent:
            return
        else:
            sound_thread = threading.Thread(target=self.sounds[i].play)
            sound_thread.start()





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
        for i, j in self.marks.items():
            print(f"{i:<6} --- {j}")
        # print(tabulate(self.marks.items()))
        # print("stopped sw")
        self.ended_at = time.monotonic()
        return int(self.ended_at - self.started_at)


class Alarm:

    def __init__(self, hrs, mins, sec=0):
        self.timer = Timer()
        self.timer.start(hrs=hrs, mins=mins, sec=sec, name='alarm')
        # you can implement better approach of letting tgread to go to sleep for some time.
        # and also alarm for specific dates.


class Pomodoro:
    timings = ((10, 5, 7), (1500, 300, 600), (3000, 600, 1200))

    def __init__(self, general=True):
        if general:
            self.work_sec, self.break_, self.lbreak = Pomodoro.timings[0]
        else:
            self.work_sec, self.break_, self.lbreak = Pomodoro.timings[1]
        self.timer = Timer(volume= 0.1)
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
        self.timer.start(sec, name='pomodoro-b')

    def pause(self):
        self.timer.pause()

    def resume(self):
        self.timer.resume()

    def reset(self):
        self.timer.remaining = 0
        self.counter = 0
        self.start()


class TaskReminder:
    def __init__(self):
        #  why you should not use login credentials to use these features.
        #  identify user because each user will have their own tasks.
        #  add, remove, update, markcomplete(toggle, status on/off)
        self.db = sqlite3.connect('memory.db')
        self.cur = self.db.cursor()
        self.columns = 'id | name | time | date | status'

    def add(self, task_name, remind_at=None):
        if remind_at:
            task_date = remind_at[0]
            task_time = remind_at[1]
            print(f'{task_date[0] + task_time[0] + task_name[0]},{task_name},{task_time},{task_date}')# taskid, taskname, time, data, status 0,1, -1 -1 if deadline is crossed.
            self.cur.execute(f"INSERT INTO Tasks values ('{task_date[0] + task_time[0] + task_name[0]}','{task_name}','{task_time}','{task_date}',0)")
            if input(f"""Confirm(y/n)
             Name:{task_name}
             Date:{task_date}
             Time:{task_time}:  """).strip().lower() == 'y':
                self.db.commit()
            else:
                print("Re-add the task. check whether the re_add is correct")

    def show_task(self, command=None):
        print("The columns are: ", self.columns)
        #  this can be used to list all the tasks which are incomplete like 'where status = -1' or for other reasons.
        if not command:
            command = input("Enter the sql command: ").strip().lower()
        # do not forget to preprocess the input like striping the whitespaces.
        if command.startswith('select'):
            self.cur.execute(command)
            for i in self.cur:  # if this is not working try fetchone, fetchmany, fetchall.
                print(i)

    def remove(self, task_name):
        self.show_task(f"select * from Tasks where name = '{task_name}'")
        # remove tasks based on different conditions like name, time, etc.
        task_id = input("enter the task id from the above displayed tasks...")
        self.cur.execute(f"Delete from Tasks where id = '{task_id}'")
        if input(f"Confirm (y/n) to delete {task_name}").strip().lower() == 'y':
            self.db.commit()
        else:
            print("Something went wrong with deleting the task. please try again. ! or .")  # This is a very straight
            # forward method try a different efficient method.

    def update(self, command):
        self.show_task()
        #print("The columns are: ", self.columns)
        self.cur.execute(command)
        self.db.commit()
        # write a method that checks the given command so that nothing wrong will happen.

    def mark_complete(self, task_name, inverse = False):
        self.show_task(f"select * from Tasks where name = '{task_name}'")
        task_id = input("enter the task id from above displayed tasks: ")
        if not inverse:
            self.cur.execute(f"update Tasks set status = 1 where id ='{task_id}'")
        else:
            self.cur.execute(f"update Tasks set status = 0 where id ='{task_id}'")
        self.db.commit()
    def mark_uncompleted(self):
        pass
        #  develop this method to tell that a specific task has crossed its deadline.


class TimeAllocated:
    """Actually, to learn the basics of any skill we should at least allocate 20hrs."""

    def __init__(self, current_skill=None):
        self.db = sqlite3.connect("memory.db")
        self.cur = self.db.cursor()
        self.stopwatch = Stopwatch()
        self.current_skill = current_skill
        if current_skill:
            self.cur.execute(f"Select allocated from Skills_time where skill = '{current_skill}'")
            if self.cur.fetchone()[0] > '20':
                pass
                # raise alarm that reached 20hrs.
        # do not forget t0 set default value for allocated time as 0
        # allocate primary key for each skill like id so that it would be easy to insert or manipulate it.

    def append(self, skill_name):
        self.cur.execute("INSERT INTO SKILLS_TIME (skill) VALUES (?)", (skill_name,))
        self.current_skill = skill_name
        #self.db.commit()  # add general functionality that confirms whether the changes should be commited or not. like
        # by showing what changes has been made.

    @staticmethod
    def add_time(total, previous):
        """this method is used to add the time like if previous is 2:10 now is 10 it should be like 2:20. """
        hrs, mins = tuple(map(int, previous.split(':')))
        hrs_and_remaining = divmod(total, 3600)
        return hrs + hrs_and_remaining[0], f"{hrs + hrs_and_remaining[0]}:{hrs_and_remaining[1] // 60}"

    def start_session(self):
        self.stopwatch.start()
        self.session_started_at = time.strftime('%d|%m|%y  %H:%M')

    def end_session(self):
        total_spent_time = self.stopwatch.stop()
        self.session_ended_at = time.strftime("%H:%M")  # set some default values or else previous time do not come
        previous, sessions = self.cur.execute(f"SELECT allocated, sessions FROM Skills_Time where skill = '{self.current_skill}'").fetchone()
        sessions = json.loads(sessions)
        allocated_till_now, incremented = TimeAllocated.add_time(total_spent_time, previous)
        # can static method be called with self or class method.
        sessions.append(
            f"{self.session_started_at} to {self.session_ended_at}")  # use json to store tuple or list ds sessions.
        # here again convert to normal list and update and convert to list or tuple and add it to db.
        sessions = json.dumps(sessions)
        self.cur.execute(f"UPDATE Skills_time set allocated = '{incremented}',sessions = '{sessions}'")
        # TODO YOU MISSED TO COMMIT  CHANGES IN THIS SECTION.
        # TODO IF BLOCK AT LINE 267 MAY NOT WORK BECAUSE ALLOCADTED_TILL_NOW HAS SOME ISSUES IN ADD_TIME FUNC LIKE
        #  DIVMOD RETURN TUPLE AND YOU ARE TRYING TO ADD IT.
        if allocated_till_now >= 20:
            pass # make_sound('completed 20hrs...')

