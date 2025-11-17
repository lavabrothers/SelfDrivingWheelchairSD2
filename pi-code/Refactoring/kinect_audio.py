#!/usr/bin/env python3

# File: kinect_audio.py
# Module for initializing the Kinect V2 microphone and
# providing an on-demand, blocking command listener.

import time
import speech_recognition as sr

class KinectAudioListener:
    """
    Handles on-demand listening for specific commands
    using the Kinect's microphone array.
    """
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.kinect_mic_index = None

    def initialize_listener(self):
        """
        Finds the Kinect microphone and adjusts for ambient noise.
        """
        print("Initializing Kinect Audio...")
        
        # 1. Find the Kinect microphone index
        print("Searching for Kinect microphone...")
        mic_names = sr.Microphone.list_microphone_names()
        for index, name in enumerate(mic_names):
            if "Sensor" in name or "Kinect" in name:
                self.kinect_mic_index = index
                print(f"Found Kinect mic at index {index} ('{name}').")
                break
        
        if self.kinect_mic_index is None:
            print("Error: Could not find Kinect V2 microphone.")
            print("Please check 'arecord -l' and mic_names list:")
            print(mic_names)
            return False

        # 2. Set up the microphone instance
        self.microphone = sr.Microphone(device_index=self.kinect_mic_index)

        # 3. Adjust for ambient noise
        print("Adjusting for ambient noise... Please be quiet for 2 seconds.")
        try:
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
            print("Audio initialized and calibrated. ✅")
            return True
        except Exception as e:
            print(f"Error during ambient noise adjustment: {e}")
            return False

    def listen_for_command(self, listen_duration=4):
        """
        Listens for a single command and returns a parsed string.
        This is a BLOCKING function. Run it in a thread.
        Returns: "follow", "cruise", "stop", or None
        """
        if not self.microphone:
            print("Error: Audio listener not initialized.")
            return None

        print(f"--- 🗣️  Listening for command... (max {listen_duration}s) ---")
        try:
            with self.microphone as source:
                # Listen for audio from the microphone
                audio_data = self.recognizer.listen(
                    source, 
                    timeout=listen_duration,      # Max time to wait for speech to start
                    phrase_time_limit=3.0 # Max length of the speech
                )
            
            # Recognize the audio using offline PocketSphinx
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
            print("Sphinx could not understand audio.")
            return None
        except sr.RequestError as e:
            print(f"PocketSphinx error: {e}")
            return None
        except Exception as e:
            print(f"An unexpected error occurred in listen_for_command: {e}")
            return None

    def shutdown_listener(self):
        """Shuts down the audio listener."""
        print("\nShutting down audio listener...")
        # No specific hardware to close, PyAudio's 'with' block handles the stream.
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