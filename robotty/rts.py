import time

class RTS(object):
    def __init__(self, freq):
        self.freq = freq

    def start(self):
        self.is_running = True
        while self.is_running:
            tic = time.time()
            self.update()
            toc = time.time()
            remaining = 1 / self.freq - (toc - tic)
            if (remaining > 0):
                time.sleep(remaining)
            else:
                print("Warning! Update took %.2f s too long!".format(-remaining))

    def stop(self):
        self.is_running = False

    def update(self):
        raise NotImplementedError()
