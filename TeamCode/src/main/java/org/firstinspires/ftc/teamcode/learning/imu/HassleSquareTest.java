package org.firstinspires.ftc.teamcode.learning.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.minibot.Autonomous.BasicMiniBotMeccanum;

@Autonomous(name="Hassle Square Drive", group="Auto")
//@Disabled

// This opMode uses the encodes and the IMU to move the robot in a square pattern.
// Copy this opmode, rename and edit the dyroDrive and gyroTurn lines to create
// a unique path.

public class HassleSquareTest extends BasicHassleMecanum {


    @Override
    public void runOpMode() {

        int squarelength = 5;

        // init subsystem hardware
        robot.init(hardwareMap, false);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Init gyro parameters then calibrate
        robot.imu.initialize(parameters);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Encoder rest is handled in the Drivetrain init in Drivetrain class

        // Calibrate gyro

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        /** Wait for the game to begin */


        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////

        drivetime.reset(); // reset because time starts when TF starts and time is up before we can call gyroDrive
        //Drive paths are initially all the same to get to the shooter location
        gyroDrive(DRIVE_SPEED, squarelength, 0.0, 10);
        gyroTurn(TURN_SPEED, 90, 10);
        gyroDrive(DRIVE_SPEED, squarelength, 90, 3);
        gyroTurn(TURN_SPEED, 180, 3);
        gyroDrive(DRIVE_SPEED, squarelength, 180, 3);
        gyroTurn(TURN_SPEED, -90, 3);
        gyroDrive(DRIVE_SPEED, squarelength, -90, 3);
        gyroTurn(TURN_SPEED, 0, 3);
        gyroDrive(DRIVE_SPEED,.5,0,1);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        sleep(1000);


    }
}