#!/usr/bin/env python3

# File: kinect_audio.py
# Module for initializing the Kinect V2 microphone and
# running a background speech recognition thread.

import time
import speech_recognition as sr
import threading

# --- Module-level Globals ---
listener_instance = None
shutdown_flag = threading.Event()

class KinectAudioListener:
    """
    Handles background listening and keyword spotting 
    using the Kinect's microphone array.
    """
    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = None
        self.kinect_mic_index = None
        self.stop_callback = None
        self.stop_listening_function = None # This is a function returned by listen_in_background()
        self.listener_thread = None

    def initialize_listener(self):
        """
        Finds the Kinect microphone and adjusts for ambient noise.
        """
        print("Initializing Kinect Audio...")
        
        # 1. Find the Kinect microphone index
        print("Searching for Kinect microphone...")
        mic_names = sr.Microphone.list_microphone_names()
        for index, name in enumerate(mic_names):
            # Your 'arecord -l' output showed "Sensor [Xbox NUI Sensor]"
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
                # We use a 2-second duration to get a good sample.
                self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
            print("Audio initialized and calibrated. ✅")
            return True
        except Exception as e:
            print(f"Error during ambient noise adjustment: {e}")
            return False

    def _background_listener_loop(self):
        """
        This is the core function that runs in its own thread.
        It continuously listens and processes audio.
        """
        # NO 'with' statement. The listen_in_background function handles the stream.
        self.stop_listening_function = self.recognizer.listen_in_background(
            self.microphone, self._process_audio_callback,
            phrase_time_limit=3.0 # Listen for up to 3 seconds of speech
        )
        print("Audio listener started in background...")
        
        # Keep this thread alive until the shutdown flag is set
        shutdown_flag.wait()
        
        # Stop the background listener when shutting down
        if self.stop_listening_function:
            self.stop_listening_function(wait_for_stop=False)
        print("Audio listener thread has stopped.")


    def _process_audio_callback(self, recognizer, audio_data):
        """
        This function is called by listen_in_background() 
        in a separate thread *every time* it hears speech.
        """
        # We use PocketSphinx for fast, offline recognition
        # It's great for simple keywords like "stop"
        try:
            text = recognizer.recognize_sphinx(audio_data).lower()
            
            if text.strip(): # Check if text is not empty
                print(f"Heard: '{text}' ", end='\r')

            # --- This is the keyword logic ---
            if "stop" in text:
                print("\n*** 'STOP' command detected! ***")
                if self.stop_callback:
                    # Call the function that was passed in from the main script
                    self.stop_callback()
                    
        except sr.UnknownValueError:
            # This is normal, just means it couldn't understand the audio
            pass 
        except sr.RequestError as e:
            print(f"PocketSphinx error: {e}")
        except Exception as e:
            print(f"Audio processing error: {e}")


    def start_listening(self, callback_on_stop):
        """
        Starts the background listening thread.
        
        :param callback_on_stop: The function to call when 'stop' is heard.
        """
        if not self.microphone:
            print("Error: Must call initialize_listener() first.")
            return False
        
        self.stop_callback = callback_on_stop
        
        # Clear the shutdown flag (in case it was set before)
        shutdown_flag.clear()
        
        # Start the background listener in a new daemon thread
        self.listener_thread = threading.Thread(
            target=self._background_listener_loop,
            daemon=True # Daemon thread will exit when main script exits
        )
        self.listener_thread.start()
        return True

    def shutdown_listener(self):
        """
Only to be used for testing, the thread is a daemon, but
        this provides a clean way to stop it.
        """
        print("\nShutting down audio listener...")
        shutdown_flag.set() # Signal the thread to stop
        if self.listener_thread:
            self.listener_thread.join(timeout=2.0) # Wait for it to finish
        print("Audio listener shut down.")


# --- Test Mode ---
if __name__ == "__main__":
    print("Running kinect_audio.py in TEST MODE.")
    
    def my_test_callback():
        print("\n--- MAIN SCRIPT: STOP CALLBACK TRIGGERED! ---")
        # In a real app, you might set a global flag here
        
    listener = KinectAudioListener()
    
    if listener.initialize_listener():
        listener.start_listening(my_test_callback)
        
        print("\nTest mode running. Say 'stop' to trigger the callback.")
        print("The listener is running in the background.")
        print("Press Ctrl+C to exit.")
        
        try:
            # Keep the main thread alive
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nCtrl+C pressed. Shutting down.")
        finally:
            listener.shutdown_listener()