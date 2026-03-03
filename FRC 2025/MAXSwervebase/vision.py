from cscore import CameraServer

def main():
    CameraServer.enableLogging()
    camera1 = CameraServer.startAutomaticCapture(0)
    camera2 = CameraServer.startAutomaticCapture(1)

    camera1.setFPS(30)
    camera2.setFPS(30)
    CameraServer.waitForever()