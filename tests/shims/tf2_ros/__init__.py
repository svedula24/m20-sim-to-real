SENT = []   # every TransformStamped broadcast, in order

class TransformBroadcaster:
    def __init__(self, node): pass
    def sendTransform(self, t):
        SENT.extend(t if isinstance(t, list) else [t])

class Buffer:                 # enough for a node to construct; lookups are not simulated
    pass

class TransformListener:
    def __init__(self, buffer, node): pass
