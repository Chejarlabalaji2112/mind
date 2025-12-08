
from mind.simulation.scripts.mind_engine import MujocoBackend 

BOOT_VIDEO_PATH = "/home/badri/mine/hitomi/mind/src/mind/simulation/media/videos/pupil_boot.mp4"
import threading
import time

def main():

    # 1. Initialize the backend
    bot = MujocoBackend()

 
    # 2. Create the thread explicitly in MAIN
    # We pass 'bot.run' as the target function
    sim_thread = threading.Thread(target=bot.run, daemon=True, name="MuJoCo-Loop")

    # 3. Start the thread
    print("Main: Starting simulation thread...")
    sim_thread.start()

    # 4. Wait for the simulation to be fully initialized (Optional but recommended)
    print("Main: Waiting for viewer...")
    if bot.wait_until_ready(timeout=10):
        print("Main: Robot is ready!")
    else:
        print("Main: Timed out waiting for robot.")
        return

    # 5. Send Commands from Main Thread
    time.sleep(1)
    bot.wake_up()

    time.sleep(4)
    bot.play_video(BOOT_VIDEO_PATH)
    

    # Keep main thread alive or do other logic
    try:
        while sim_thread.is_alive():

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Main: Stopping...")
         
        bot.stop()
        sim_thread.join()
        print("Main: Exited.")

if __name__ == "__main__":
    main()