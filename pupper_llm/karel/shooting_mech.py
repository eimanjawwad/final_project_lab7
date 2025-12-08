from gpiozero import LED
from time import sleep

PIN = 16   # GPIO pin number

def shoot_once():
    out = LED(PIN)

    print("Sending 1-second HIGH pulse on GPIO 16...")
    out.on()           # HIGH (3.3V)
    sleep(1.0)         # keep it high for 1 second
    out.off()          # LOW (0V)
    print("Pulse complete. GPIO 16 is LOW.")

