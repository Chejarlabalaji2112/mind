
from mind.utils import setup_logger
from mind.utils.time_conversions import parse_time_string

logger = setup_logger(__name__)

class SymbolicHandler:
    def __init__(self, tool_instances, presenters, robot_controller):
        self.tools = tool_instances
        self.presenters  = presenters
        self.robot_controller = robot_controller

    async def process(self, command: str):
        """
        Parses and executes a symbolic command starting with '/'.
        Format: /request action [args...]
        Example: /timer start 10m
        """
        parts = command.strip().split()
        if not parts:
            return

        # Remove the leading '/' and get tool name
        request = parts[0][1:].lower()
        action = parts[1].lower() if len(parts) > 1 else None
        args = parts[2:]

        try:
            if request == "timer":
                self._handle_timer(action, args)
            elif request == "stopwatch":
                self._handle_stopwatch(action, args)
            elif request == "pomodoro":
                self._handle_pomodoro(action, args)
            elif request == "sktracker":
                self._handle_skills_tracker(action, args)
            elif request == "clear":
                self.robot_controller.show_clock = True
                for presenter in self.presenters:
                    presenter.clear()
            elif request == "h_mov":
                if action == "yes":
                    self.robot_controller.nod_yes()
                if action == "no":
                    self.robot_controller.shake_no()
            elif request == "chat_name":
                logger.info("requesting to set a chat id for the chat.")
                return " ".join(parts[1:]).lower()
                    
            else:
                logger.warning(f"Unknown symbolic tool: {request}")
        except Exception as e:
            logger.error(f"Error executing symbolic command '{command}': {e}")

    def _handle_timer(self, action, args):
        timer = self.tools.timer
        if action == "start":
            duration = parse_time_string(args[0]) if args else 0
            timer.start(duration)
        elif action == "pause":
            timer.pause()
        elif action == "resume":
            timer.resume()
        elif action == "stop":
            timer.stop()
        elif action == "reset":
            timer.reset()

    def _handle_stopwatch(self, action, args):
        stopwatch = self.tools.stopwatch
        if action == "start":
            stopwatch.start()
        elif action == "pause":
            stopwatch.pause()
        elif action == "resume":
            stopwatch.resume()
        elif action == "stop":
            stopwatch.stop()
        elif action == "reset":
            stopwatch.reset()
        elif action == "lap":
            stopwatch.lap()

    def _handle_pomodoro(self, action, args):
        pomodoro = self.tools.pomodoro
        if action == "start":
            pom_type = args[0] if args else "short"
            pomodoro.start(pom_type=pom_type)
        elif action == "pause":
            pomodoro.pause()
        elif action == "resume":
            pomodoro.resume()
        elif action == "stop":
            pomodoro.stop()
        elif action == "reset":
            pomodoro.reset()

    def _handle_skills_tracker(self, action, args):
        tracker = self.tools.skillstracker
        if not action:
            return

        if action == "create":
            if args: tracker.create_skill(" ".join(args))
        elif action == "start":
            if args: tracker.start_session(" ".join(args))
        elif action == "end":
            if args: tracker.end_session(" ".join(args))
        elif action == "delete":
            if args: tracker.delete_skill(" ".join(args))
        elif action == "reset":
            if args: tracker.reset_skill(" ".join(args))
        elif action == "list":
            skills = tracker.get_all()
            # Note: Output handling for lists/data might need a return mechanism 
            # or rely on logs/console for this specific implementation.
            logger.info(f"Skills: {skills}")
