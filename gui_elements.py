def stream_to_display(value):
    cap = cv.VideoCapture(value)
    canvas_img = canvas.create_image(0, 0, anchor=tk.NW)
    def frame_update():
        ret, frame = cap.read()
        if ret:
            frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            frame_resized = cv.resize(frame, (width, height))
            img = Image.fromarray(frame_resized)
            imgtk = ImageTk.PhotoImage(image=img)
            canvas.itemconfig(canvas_img, image = imgtk)
            canvas.imgtk = imgtk
        root.after(100, frame_update)

    frame_update()
    def on_closing():
        print("closing...")
        cap.release()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)

def display_text(text):
    canvas.create_text(width//2, height//2, text=text, fill='lightblue', font=("Arial", 20))

def format_time(seconds):
    mins, secs = divmod(seconds, 60)
    hrs, mins = divmod(mins, 60)
    return f"{hrs:02d}:{mins:02d}:{secs:02d}"
def update_countdown():
    remaining_time = timer.remaining
    formatted_time = format_time(remaining_time)
    canvas.itemconfig(countdown_text, text=formatted_time)
    if remaining_time > 0:
        root.after(1000, update_countdown)
    else:
        canvas.itemconfig(countdown_text, text="time's up")
def start_timer():
    timer.start(90)
    update_countdown()

start_button = tk.Button(root, text="start", command = start_timer)
start_button.pack(pady=10)

def pause_timer():
    timer.pause()

pause_button = tk.Button(root, text="pause", command=pause_timer)
pause_button.pack(pady=10)
def resume_timer():
    timer.resume()
resume_button = tk.Button(root, text="resume", command=resume_timer)
resume_button.pack(pady=10)
def on_closing():
    print("closing....")
    timer.kill()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
#stream_to_display('/home/badri/Videos/Camera/testvideo.webm')