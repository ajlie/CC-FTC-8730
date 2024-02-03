package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.OpenCvPipeline.BlackWhiteDetector;

public class TeamElementSubsystem {
    OpenCvCamera camera;
    static BlackWhiteDetector BlackWhiteDetector;
    int camW = 1280;
    int camH = 720;

    int zone = 1;

    public TeamElementSubsystem(HardwareMap hardwareMap){
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        BlackWhiteDetector = new BlackWhiteDetector();

        camera.setPipeline(BlackWhiteDetector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(camW, camH, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }



    public int elementDetection(Telemetry telemetry) {
        zone = splitAveragePipeline.get_element_zone();
        telemetry.addData("Element Zone", zone);
        return zone;
    }

}
