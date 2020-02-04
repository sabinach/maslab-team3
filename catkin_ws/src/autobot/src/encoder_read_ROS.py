#!/usr/bin/env python

from tamproxy import ROSSketch, SyncedSketch, Timer
from tamproxy.devices import Encoder

# Prints a quadrature encoder's position
class EncoderRead(ROSSketch):

    pins = 14, 15

    def setup(self):
        self.encoder = Encoder(self.tamp, *self.pins, continuous=True)
        self.timer = Timer()

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            print(self.encoder.val)

if __name__ == "__main__":
    sketch = EncoderRead()
    sketch.run()
