"""
kinect_audio.py

This module provides functionality for initializing and listening to voice commands
using the Kinect V2's microphone array. It leverages the `speech_recognition` library
with an offline PocketSphinx recognizer to detect specific keywords like "follow",
"cruise", and "stop".

The `KinectAudioListener` class encapsulates the microphone setup, ambient noise
adjustment, and the blocking command listening process, making it suitable for
integration into a multi-threaded or asynchronous application where voice control
is desired.

Dependencies:
- speech_recognition: A library for performing speech recognition, with support
                      for various engines including PocketSphinx.
"""

import time
import speech_recognition as sr

class KinectAudioListener:
    """
    Handles on-demand listening for specific voice commands using the Kinect V2's
    microphone array.

    This class manages the initialization of the speech recognizer and microphone,
    ambient noise calibration, and the process of listening for and interpreting
    predefined voice commands.
    """
    def __init__(self):
        """
        Initializes the KinectAudioListener.

        Sets up a `speech_recognition.Recognizer` instance and placeholders for
        the microphone object and its index.
        """
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.kinect_mic_index = None

    def initialize_listener(self) -> bool:
        """
        Finds the Kinect V2 microphone, sets it up, and adjusts for ambient noise.

        This method iterates through available microphones to identify the Kinect
        device, then configures the `speech_recognition.Microphone` instance.
        It performs an ambient noise adjustment to improve recognition accuracy.

        Returns:
            bool: True if the listener was successfully initialized and calibrated,
                  False otherwise.
        """
        print("Initializing Kinect Audio...")
        
        # 1. Find the Kinect microphone index by name.
        print("Searching for Kinect microphone...")
        mic_names = sr.Microphone.list_microphone_names()
        for index, name in enumerate(mic_names):
            if "Sensor" in name or "Kinect" in name: # Common identifiers for Kinect mic.
                self.kinect_mic_index = index
                print(f"Found Kinect mic at index {index} ('{name}').")
                break
        
        if self.kinect_mic_index is None:
            print("Error: Could not find Kinect V2 microphone.")
            print("Please check 'arecord -l' and mic_names list:")
            print(mic_names)
            return False

        # 2. Set up the microphone instance using the identified index.
        self.microphone = sr.Microphone(device_index=self.kinect_mic_index)

        # 3. Adjust for ambient noise to improve recognition accuracy.
        print("Adjusting for ambient noise... Please be quiet for 2 seconds.")
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
            print("Audio initialized and calibrated. ✅")
            return True
        except Exception as e:
            print(f"Error during ambient noise adjustment: {e}")
            return False

    def listen_for_command(self, listen_duration: int = 4) -> str | None:
        """
        Listens for a single voice command for a specified duration and attempts
        to recognize it. This is a BLOCKING function and should typically be run
        in a separate thread or asyncio executor.

        Args:
            listen_duration (int): The maximum number of seconds to wait for speech
                                   to start. Defaults to 4 seconds.

        Returns:
            str | None: A recognized command string ("follow", "cruise", "stop")
                        if successful, otherwise None.
        """
        if not self.microphone:
            print("Error: Audio listener not initialized.")
            return None

        print(f"--- 🗣️  Listening for command... (max {listen_duration}s) ---")
        try:
            with self.microphone as source:
                # Listen for audio from the microphone, with timeouts.
                audio_data = self.recognizer.listen(
                    source, 
                    timeout=listen_duration,      # Max time to wait for speech to begin.
                    phrase_time_limit=3.0         # Max length of the speech phrase to consider.
                )
            
            # Recognize the audio using the offline PocketSphinx engine.
            print("Processing audio...")
            text = self.recognizer.recognize_sphinx(audio_data).lower()
            print(f"Heard: '{text}'")
            
            # --- Parse for known commands ---
            if "follow" in text:
                return "follow"
            if "cruise" in text:
                return "cruise"
            if "stop" in text:
                return "stop"
            
            print("Command not recognized.")
            return None
            
        except sr.WaitTimeoutError:
            print("No speech detected within timeout.")
            return None
        except sr.UnknownValueError:
            print("PocketSphinx could not understand audio.")
            return None
        except sr.RequestError as e:
            print(f"PocketSphinx error: {e}")
            return None
        except Exception as e:
            print(f"An unexpected error occurred in listen_for_command: {e}")
            return None

    def shutdown_listener(self):
        """
        Shuts down the audio listener.

        This method performs any necessary cleanup for the audio resources.
        (For `speech_recognition`, the `with` block for `microphone` handles stream closure).
        """
        print("\nShutting down audio listener...")
        print("Audio listener shut down.")


# --- Test Mode ---
if __name__ == "__main__":
    print("Running kinect_audio.py in TEST MODE.")
    
    listener = KinectAudioListener()
    
    if listener.initialize_listener():
        print("\nTest: Please say a command (e.g., 'follow', 'cruise', 'stop').")
        
        command = listener.listen_for_command()
        
        if command:
            print(f"\n--- TEST: COMMAND RECOGNIZED: '{command}' ---")
        else:
            print("\n--- TEST: NO COMMAND RECOGNIZED ---")
            
    listener.shutdown_listener()
