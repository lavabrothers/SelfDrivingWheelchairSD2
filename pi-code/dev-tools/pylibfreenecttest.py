import numpy as np
import cv2
import sys
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame, FrameMap

try:
    cv2.ogl.setUseOpenGl(False)
except:
    pass

fn = Freenect2()
num_devices = fn.enumerateDevices()
if num_devices == 0:
    print("Error: No Kinect v2 devices found!")
    sys.exit(1)

serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial)

types = FrameType.Color | FrameType.Depth
listener = SyncMultiFrameListener(types)

device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)

print("Starting device...")
device.start()

registration = Registration(device.getIrCameraParams(),
                            device.getColorCameraParams())

frames = FrameMap()

print("Device started. Press 'q' in the video windows to quit.")

while True:
    if listener.waitForNewFrame(frames, 10 * 1000):
        
        color_frame = frames[FrameType.Color]
        depth_frame = frames[FrameType.Depth]

        # --- Get the RAW Color Image (Large) ---
        color_image = color_frame.asarray()[:,:,:3]
        color_image_small = cv2.resize(color_image, (960, 540))
        cv2.imshow("Color (Full)", color_image_small)

        # --- Get the RAW Depth Image (Small) ---
        raw_depth_image = depth_frame.asarray(np.float32)
        # Normalize for visualization
        raw_depth_viz = cv2.normalize(raw_depth_image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        cv2.imshow("Raw Depth", raw_depth_viz)

        # --- Get the REGISTERED (Aligned) Image ---
        # These are the output frames
        undistorted = Frame(512, 424, 4)
        registered = Frame(512, 424, 4)
        
        # This call populates 'registered' with a 512x424 COLOR image
        # and 'undistorted' with a 512x424 corrected DEPTH image.
        registration.apply(color_frame, depth_frame, undistorted, registered)

        # Convert the REGISTERED frame to a numpy array
        # This is a COLOR image, so we use np.uint8
        registered_image = registered.asarray(np.uint8)[:,:,:3]
        cv2.imshow("Registered Color (Aligned with Depth)", registered_image)
        # --- End of Fix ---

        listener.release(frames)
    
    else:
        print("Timeout! Waiting for new frames...")
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

print("Stopping device...")
device.stop()
device.close()
cv2.destroyAllWindows()
print("Script finished.")