from utils import RoboEyes, DEFAULT, TIRED, ANGRY, HAPPY, N, NE, E, SE, S, SW, W, NW, CENTER
from .cv_display import CvDisplay

display = CvDisplay()
eyes = RoboEyes(display, 512, 512)
eyes.is_active = True
eyes.begin()
eyes.set_mood(HAPPY)
eyes.set_idle_mode(True)
eyes.set_auto_blink(True)    

import time
start_time = time.time()
while time.time() - start_time < 10:  # Run for 10s
    if time.time() - start_time > 3:
       pass
    eyes.update()
    display.update()
    time.sleep(1/30)

    