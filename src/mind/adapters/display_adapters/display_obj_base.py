from ports.act_port import Presenter

class DisplayObj(Presenter):
    """Base class for display objects used in AVOrchestrator.
    
    Must implement:
        - put_frame(frame): Display a single video frame.
        - update(): Update the display (if needed).
    """
    def show(self, frame):
        raise NotImplementedError("put_frame() must be implemented by subclasses.")

    def update(self):
        raise NotImplementedError("update() must be implemented by subclasses.")