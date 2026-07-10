import threading
import lager.parameters as params
import RPi.GPIO as _GPIO


class LED:
    """Controls a single status LED on a GPIO pin.

    States
    ------
    LED_STATE_OFF   : LED off
    LED_STATE_ON    : LED continuously on
    LED_STATE_BLINK : LED blinks at *blink_frequency* Hz
    """

    def __init__(self, pin: int, blink_frequency: float = 2.0):
        self.pin = pin
        self.blink_frequency = blink_frequency
        self._state = params.LED_STATE_OFF
        self._stop_blink = threading.Event()
        self._blink_thread = None

        _GPIO.setmode(_GPIO.BCM)
        _GPIO.setup(self.pin, _GPIO.OUT, initial=_GPIO.LOW)

    def set(self, state: int, blink_frequency: float = None) -> None:
        """Set the LED state (LED_STATE_OFF / ON / BLINK)."""
        if blink_frequency is not None:
            self.blink_frequency = blink_frequency

        # stop any running blink thread before changing state
        self._stop_blink_thread()

        self._state = state

        if state == params.LED_STATE_ON:
            self._write(True)
        elif state == params.LED_STATE_OFF:
            self._write(False)
        elif state == params.LED_STATE_BLINK:
            self._stop_blink.clear()
            self._blink_thread = threading.Thread(
                target=self._blink_loop, daemon=True, name="led-blink"
            )
            self._blink_thread.start()

    def cleanup(self) -> None:
        """Turn off the LED and release the GPIO pin."""
        self._stop_blink_thread()
        self._write(False)
        _GPIO.cleanup(self.pin)

    def _write(self, value: bool) -> None:
        _GPIO.output(self.pin, _GPIO.HIGH if value else _GPIO.LOW)

    def _blink_loop(self) -> None:
        half_period = 1.0 / (2.0 * self.blink_frequency)
        while not self._stop_blink.is_set():
            self._write(True)
            if self._stop_blink.wait(half_period):
                break
            self._write(False)
            self._stop_blink.wait(half_period)

    def _stop_blink_thread(self) -> None:
        if self._blink_thread is not None and self._blink_thread.is_alive():
            self._stop_blink.set()
            self._blink_thread.join()
        self._blink_thread = None

    def __del__(self) -> None:
        try:
            self.cleanup()
        except Exception:
            pass
