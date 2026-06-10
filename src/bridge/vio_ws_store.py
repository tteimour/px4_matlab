"""Latest-message store for roslibpy subscriptions polled from MATLAB.

roslibpy delivers messages on Twisted's background thread; MATLAB must not
be called back from there. Instead this tiny callable stores the newest
message (plain dict) and MATLAB polls `msg` from its own thread each frame.
Attribute assignment is atomic under the GIL, so no locking is needed for
a single-writer / single-reader latest-value exchange.
"""


class VioWsStore:
    def __init__(self):
        self.msg = None
        self.count = 0

    def __call__(self, message):
        self.msg = message
        self.count += 1
