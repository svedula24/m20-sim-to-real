class _Auto:
    """Nested message fields spring into existence on first access, so node code can write
    msg.header.stamp.sec = ... and tests can read it back. Not a real message type."""
    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        v = _Auto(); object.__setattr__(self, k, v); return v

class _V3:
    def __init__(self): self.x = 0.0; self.y = 0.0; self.z = 0.0
class Twist:
    def __init__(self): self.linear = _V3(); self.angular = _V3()
class PoseStamped(_Auto):
    pass
class TransformStamped(_Auto):
    pass
