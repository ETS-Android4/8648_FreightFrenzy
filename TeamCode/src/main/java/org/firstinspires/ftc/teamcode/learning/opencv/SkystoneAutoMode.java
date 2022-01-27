package org.firstinspires.ftc.teamcode.learning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.enums.Alliance;
import org.firstinspires.ftc.teamcode.enums.Barcode;
import org.firstinspires.ftc.teamcode.enums.StartSide;
import org.firstinspires.ftc.teamcode.learning.FreightFrenzyTSEPipeline_EXP;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name="Skystone Detector", group="Auto")
//@Disabled
public class SkystoneAutoMode extends LinearOpMode {
    OpenCvCamera webcam;
    FreightFrenzyTSEPipeline_EXP pipeline;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
      //  FreightFrenzyTSEPipeline_EXP detector = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.BLUE, StartSide.WAREHOUSE);
        pipeline = new FreightFrenzyTSEPipeline_EXP(telemetry, Alliance.BLUE, StartSide.WAREHOUSE);
        //webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getLocation());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        //waitForStart();
        telemetry.addData("Snapshot post-START analysis", pipeline.getLocation());
        telemetry.update();

       switch (pipeline.getLocation()) {
            case LEFT:
                // ...
                break;
            case CENTER:
                // ...
                break;
            case RIGHT:
                // ...
        }
       //webcam.stopStreaming();
       while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}
