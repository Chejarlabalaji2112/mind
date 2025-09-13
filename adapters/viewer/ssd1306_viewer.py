from ports.act_port import viewer

class SSD1306Viewer(viewer):
    """This class handles showing the output visually on an SSD1306 display which is powered by esp32 and communication through wifi. because computer can not find ip address aurtomatically, we use mdns
    """
    def __init__(self):
        self.description = "A class to handle SSD1306 display functionalities." 

