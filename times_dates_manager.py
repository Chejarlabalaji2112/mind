import sqlite3
import time
import threading


# from tabulate import tabulate


class Timer:
    def __init__(self):
        self.control_event = threading.Event()
        self.control_event.set()
        self.completed = False
        self.new = True

    def start(self, sec, mins=0, hrs=0, name=None):
        def inner_start():
            self.duration = (hrs * 3600) + (mins * 60) + sec
            self.remaining = self.duration
            self.thread = threading.Thread(target=self.run, name=name)
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
        for i, j in self.marks.items():
            print(f"{i:<6} --- {j}")
        # print(tabulate(self.marks.items()))
        # print("stopped sw")
        self.ended_at = time.monotonic()
        return int(self.ended_at- self.started_at)


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
        self.db = sqlite3.connect()
        self.cur = self.db.cursor()
        self.columns = 'id | name | time | date | status'


    def add(self, task_name, remind_at=None):
        if remind_at:
            task_date = remind_at[0]
            task_time = remind_at[1]  #  taskid, taskname, time, data, status 0,1, -1 -1 if deadline is crossed.
            self.cur.execute(f"INSERT INTO Tasks({task_date[0]+task_time[0]+task_name[0]},{task_name},{task_time},{task_date},0)")
            if input(f"""Confirm(y/n)
             Name:{task_name}
             Date:{task_date}
             Time:{task_time}""").strip().lower() == 'y':
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
        self.show_task(f'select * from Tasks where name = {task_name}')
        # remove tasks based on different conditions like name, time, etc.
        task_id = input("enter the task id from the above displayed tasks...")
        self.cur.execute(f"Delete from Tasks where id = {task_id}")
        if input(f"Confirm (y/n) to delete {task_name}").strip().lower() == 'y':
            self.db.commit()
        else:
            print("Something went wrong with deleting the task. please try again. ! or .")  # This is a very straight
            # forward method try a different efficient method.

    def update(self, command):
        self.show_task()
        print("The columns are: ", self.columns)
        self.cur.execute(command)
        # write a method that checks the given command so that nothing wrong will happen.

    def mark_complete(self, task_name):
        self.show_task(f'select * from Tasks where name = {task_name}')
        task_id = input("enter the task id from above displayed tasks...")
        self.cur.execute(f"update Tasks set status = 1 where id ={task_id}")

    def mark_uncompleted(self):
         #  develop this method to tell that a specific task has crossed its deadline.


class TimeAllocated:
    """Actually, to learn the basics of any skill we should at least allocate 20hrs."""
    def __init__(self, current_skill = None):
        self.db = sqlite3.connect("memory.db")
        self.cur = self.db.cursor()
        self.stopwatch = Stopwatch()
        self.current_skill = current_skill
        if current_skill:
            self.cur.execute(f"Select allocated_time from Skills_time where skill_name = {current_skill}")
            if self.cur.fetchone() > '20':
                pass
                # raise alarm that reached 20hrs.
        # do not forget t0 set default value for allocated time as 0
        # allocate primary key for each skill like id so that it would be easy to insert or manipulate it.

    def append(self,skill_name):
        self.cur.execute(f"INSERT INTO SKILLS_TIME({skill_name} ")
        self.db.commit() # add general functionality that confirms whether the changes should be commited or not. like
        # by showing what changes has been made.

    @staticmethod
    def add_time(total, previous):
          """this method is used to add the time like if previous is 2:10 now is 10 it should be like 2:20 """

    def start_session(self):
        self.stopwatch.start()
        self.session_started_at = self.stopwatch.started_at, time.strftime('%H:%M')

    def end_session(self):
        total = self.stopwatch.stop()
        self.session_ended_at = time.strftime("%H:%M") # set some default values or else previous time do not come
        previous, session = self.cur.execute(f"select allocated_time, session FROM Skills_Time where skill = {self.current_skill}")
        incremented = self.add_time(total, previous)
        # can static method be called with self or class method.
        session = session + self.session_ended_at # use json to store tuple or list ds session.
        # here again convert to normal list and update and convert to list or tuple and add it to db.
        self.cur.execute(f"UPDATE Skills_time set allocated_time = {incremented}, set session = {session}")







