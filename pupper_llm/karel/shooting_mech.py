import lgpio
from time import sleep

PIN = 26   # GPIO pin number

def shoot_once():
    chip = lgpio.gpiochip_open(0)  # Open GPIO chip 0
    lgpio.gpio_claim_output(chip, PIN)  # Claim GPIO pin as output

    print("Sending 1-second HIGH pulse on GPIO 26...")
    # while True:
    lgpio.gpio_write(chip, PIN, 1)           # HIGH (3.3V)
    sleep(1.0)         # keep it high for 1 second
    lgpio.gpio_write(chip, PIN, 0)          # LOW (0V)
    lgpio.gpiochip_close(chip)                     # Clean up GPIO settings
    print("Pulse complete. GPIO 26 is LOW.")
