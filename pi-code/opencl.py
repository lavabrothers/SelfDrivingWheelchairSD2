import cv2

if cv2.ocl.haveOpenCL():
    cv2.ocl.setUseOpenCL(True)
    print("OpenCL is available and enabled! ✅")
    # This line has been corrected
    print("Platform:", cv2.ocl.getPlatformsInfo()[0].name)
else:
    print("Sorry, OpenCL is not supported by this OpenCV build. ❌")