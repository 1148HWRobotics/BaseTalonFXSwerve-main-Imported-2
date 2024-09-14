package frc.Devices;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class DriverCamera {
    private UsbCamera driverCam;

    public DriverCamera() {
        this.driverCam = CameraServer.startAutomaticCapture();
        driverCam.setFPS(120);
        driverCam.setResolution(360, 200);
    }

}
