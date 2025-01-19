import tkinter as tk
#from PIL import Image, ImageTk

from times_dates_manager import Timer
#import cv2 as cv

class TimerGui:
    def __init__(self, main,canvas, width, height):
        self.main = main
        self.canvas = canvas
        self.dis_width, self.dis_height = width, height
        self.timer = Timer()
        self.left_clicked = 0

    def setup(self):
        print("enterd timergui")
        self.canvas.delete('all')
        self.left_clicked = 0
        self.canvas_text = self.canvas.create_text(self.dis_width // 2, self.dis_height // 2, text="00:00:00",
                                                   fill='lightblue', font=("Arial", 20, 'bold'))
        self.left_button = tk.Button(text="🔙", font=("Arial", 20), command=self.click_left)
        self.left_button.pack(side="left")
        self.entry = tk.Entry(width= 5)
        self.entry.pack(side="right")
        self.enter_button = tk.Button(text="🆗", font=("Arial", 20), command=self.click_ok )
        self.enter_button.pack(side="left", padx=30)

    @staticmethod
    def format_time(seconds):
        mins, secs = divmod(seconds, 60)
        hrs, mins = divmod(mins, 60)
        return f"{hrs:02d}:{mins:02d}:{secs:02d}"

    def update_countdown(self):
        remaining_time = self.timer.remaining
        self.formatted_time = self.format_time(remaining_time)
        self.canvas.itemconfig(self.canvas_text, text=self.formatted_time)
        if remaining_time > 0:
            self.main.after(1000, self.update_countdown)
        else:
            self.canvas.itemconfig(self.canvas_text, text="time's up")
    def start_timer(self, time):
        self.timer.start(time)
        self.update_countdown()

    def click_ok(self):
        if (entered := self.entry.get()):
            match entered[-1]:
                case 'm':
                    timer_sec = int(entered[:-1]) * 60 # here converting into seconds and it get convereted into appropriate inside timer.TODO
                case _:
                    timer_sec = int(entered[:-1])
        self.start_timer(timer_sec)

    def click_left(self):
        self.left_clicked += 1
        if self.left_clicked > 1:
            self.entry.destroy()
            self.enter_button.destroy()
            self.left_button.destroy()
            global _all_objects
            _all_objects[0].setup()
        self.timer.kill()


class Home:
    def __init__(self,main,canvas, width, height):
        self.main = main
        self.canvas = canvas
        self.dis_width, self.dis_height = width, height
        self.options = ["HITOMI", 'TIMER', 'POMODORO', 'STOPWATCH']


    def setup(self):
        print("home_setup")
        self.canvas.delete('all')
        self.canvas_text = self.canvas.create_text(dis_width // 2, dis_height // 2, text="HITOMI", fill='lightblue',
                                                   font=("Arial", 20, 'bold'))
        self.current =0
        self.left_button = tk.Button(text ="🔙", font=("Arial", 20), command=self.click_left)
        self.enter_button = tk.Button(text="🆗", font=("Arial", 20),command=self.select_option)
        self.right_button = tk.Button(text="🔜", font=("Arial", 20), command=self.click_right)
        self.left_button.pack(side="left")
        self.right_button.pack(side="right")
        self.enter_button.pack(side="left", padx = 30)


    def click_left(self):
        self.current -= 1
        self.current %= 4
        self.canvas.itemconfig(self.canvas_text, text=self.options[self.current])

    def click_right(self):
        self.current += 1
        self.current %= 4
        self.canvas.itemconfig(self.canvas_text, text=self.options[self.current])

    def select_option(self):
        self.left_button.destroy()
        self.enter_button.destroy()
        self.right_button.destroy()
        global _all_objects
        match self.current:
            case 1:
                _all_objects[1].setup()
            case _:
                _all_objects[0].setup()
dis_width, dis_height = 256, 256 #Width and Height of the display.

main_win = tk.Tk()
main_win.title(f"HITOMI")
main_canvas = tk.Canvas(main_win, width=dis_width, height=dis_height, bg= "black") #canvas is where our images and text will be displayed.
main_canvas.pack()

def on_closing():
    """To clean the whole window after taking confirmation or saving any data."""
    print("closing....")
    if not _all_objects[1].timer.new:
        _all_objects[1].timer.kill()
    main_win.destroy()


_all_objects = [Home(main_win, main_canvas,dis_width, dis_height), TimerGui(main_win, main_canvas, dis_width, dis_height)]
main_win.protocol("WM_DELETE_WINDOW", on_closing)
_all_objects[0].setup()
main_win.mainloop()