from machine import Pin
import rp2

@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, autopull=True, pull_thresh=2)
def quadrature():
    label("loop")
    in_(pins, 2)
    push(block)
    jmp("loop")

class Quadrature:
    def __init__(self, pin_a, pin_b, sm_id=0):
        self.pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self._position = 0
        self.last_state = None

        self.sm = rp2.StateMachine(
            sm_id,
            quadrature,
            in_base=self.pin_a,
            freq=1000000
        )
        self.sm.active(1)

    def position(self):
        transitions = {
            0b0001: +1,
            0b0010: -1,
            0b0100: -1,
            0b1000: +1
        }

        max_iters = 100  # avoid hanging if FIFO stalls
        iters = 0

        while self.sm.rx_fifo() and iters < max_iters:
            try:
                state = self.sm.get() & 0b11
            except:
                break

            if self.last_state is None:
                self.last_state = state
                continue

            transition = (self.last_state << 2) | state
            self._position += transitions.get(transition, 0)
            self.last_state = state
            iters += 1

        return self._position

    def reset(self):
        self._position = 0
        self.last_state = None

