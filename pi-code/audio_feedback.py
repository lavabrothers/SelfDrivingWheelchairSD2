"""
audio_feedback.py

This module provides asynchronous control for a beeper connected to a Raspberry Pi GPIO pin.
It allows for playing short beeps and longer alert sounds, useful for providing audio feedback
in the self-driving wheelchair system.

Dependencies:
- RPi.GPIO: For controlling Raspberry Pi GPIO pins.
- asyncio: For asynchronous operations, allowing non-blocking beep sequences.
"""

import RPi.GPIO as GPIO
import asyncio

# --- Configuration ---
BEEP_PIN = 24       # The GPIO pin connected to the beeper/speaker.
BEEP_DURATION = 0.08 # Duration of a single short beep in seconds.
PAUSE_DURATION = 0.08 # Duration of the pause between multiple short beeps in seconds.

def initialize_beeper():
    """
    Initializes the GPIO pin for the beeper.

    Sets the GPIO mode to BCM, configures the BEEP_PIN as an output,
    and ensures the beeper is initially off (LOW).

    Returns:
        bool: True if initialization was successful, False otherwise.
    """
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BEEP_PIN, GPIO.OUT)
        GPIO.output(BEEP_PIN, GPIO.LOW)
        print(f"Beeper initialized on GPIO {BEEP_PIN}")
        return True
    except Exception as e:
        print(f"Error initializing beeper: {e}")
        return False

def cleanup_beeper():
    """
    Cleans up the GPIO pin used by the beeper.

    Resets the GPIO pin configuration, releasing the resource.
    """
    print("Cleaning up beeper GPIO.")
    GPIO.cleanup(BEEP_PIN)

async def play_beep(times: int):
    """
    Plays a specified number of short beeps asynchronously.

    Each beep consists of the beeper turning ON for BEEP_DURATION,
    followed by a pause of PAUSE_DURATION.

    Args:
        times (int): The number of short beeps to play.
    """
    for _ in range(times):
        try:
            GPIO.output(BEEP_PIN, GPIO.HIGH)
            await asyncio.sleep(BEEP_DURATION)
            GPIO.output(BEEP_PIN, GPIO.LOW)
            await asyncio.sleep(PAUSE_DURATION)
        except Exception as e:
            print(f"Error during beep sequence: {e}")
            break # Stop beeping if an error occurs

async def play_long_beep():
    """
    Plays a single, longer beep asynchronously.

    The duration of this beep is four times the standard BEEP_DURATION.
    Useful for signaling alerts or important events.
    """
    try:
        GPIO.output(BEEP_PIN, GPIO.HIGH)
        await asyncio.sleep(BEEP_DURATION * 4)  # Make it 4x longer
        GPIO.output(BEEP_PIN, GPIO.LOW)
    except Exception as e:
        print(f"Error during long beep: {e}")
