package frc.robot.systems;

public class Webcam {

    public Webcam() {

    }

    public static void main(string... args){
        runWebcam();
    }

    public void runWebcam() {
        // Creates UsbCamera and MjpegServer [1] and connects them
        CameraServer.startAutomaticCapture();
        // Creates the CvSink and connects it to the UsbCamera
        CvSink cvSink = CameraServer.getVideo();
        // Creates the CvSource and MjpegServer [2] and connects them
        CvSource outputStream = CameraServer.putVideo("RobotFrontCamera", 640, 480);
    }
}
