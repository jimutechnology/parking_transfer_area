import time

class TimerDuration:
    def __init__(self, duration=0):
        self.duration = duration
        self.running = False
        self.start_time = time.time()
        self.Pause_time = time.time()

    def Start(self):
        self.running = True
        self.start_time = time.time()

    def Tick(self):
        n = time.time()
        return (n - self.start_time) * 1000 > self.duration and self.running

    def Stop(self):
        self.running = False

    def Pause(self):
        self.running = False
        self.Pause_time = time.time()

    def Resume(self):
        self.running = True
        self.duration += time.time() - self.Pause_time


