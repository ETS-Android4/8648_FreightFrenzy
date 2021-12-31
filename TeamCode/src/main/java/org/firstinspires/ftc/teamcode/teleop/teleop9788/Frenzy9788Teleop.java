package org.firstinspires.ftc.teamcode.teleop.teleop9788;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.hardwarepusbots.Frenzy9788HardwarePushbot;

@TeleOp(name="9788 Teleop", group="Pushbot")
@Disabled
public class Frenzy9788Teleop extends LinearOpMode {

    Frenzy9788HardwarePushbot robot = new Frenzy9788HardwarePushbot();
    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){


            robot.rightFront.setPower(gamepad1.right_stick_y);
            robot.rightBack.setPower(gamepad1.right_stick_y);
            robot.leftFront.setPower(-gamepad1.left_stick_y);
            robot.leftBack.setPower(-gamepad1.left_stick_y);

        if (gamepad1.dpad_left) {
            robot.leftFront.setPower(-1);
            robot.rightFront.setPower(-1);
            robot.leftBack.setPower(1);
            robot.rightBack.setPower(1);
        }
         else if (gamepad1.dpad_right) {
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(1);
            robot.leftBack.setPower(-1);
            robot.rightBack.setPower(-1);
         }
         else {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }

  /*          double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double lf, lb, rf, rb;

            lf = (y + x + rx); //forward + turn right + strafe right
            lb = (y - x + rx);
            rf = (y - x - rx);
            rb = (y + x - rx);

            if (Math.abs(lf) > 1 || Math.abs(lb) > 1 || Math.abs(rf) > 1 || Math.abs(rb) > 1) {
                double max = 0;
                max = Math.max(Math.abs(lf), Math.abs(lb));
                max = Math.max(Math.abs(rf), max);
                max = Math.max(Math.abs(rb), max);

                // scales output if y + x + rx >1
                lf /= max;
                lb /= max;
                rf /= max;
                rb /= max;

            }
            robot.leftFront.setPower(lf);
            robot.rightFront.setPower(rf);
            robot.leftBack.setPower(lb);
            robot.rightBack.setPower(rb);*/





                if (gamepad2.x)
                    robot.carousel.setPower(0.8);
                else if(gamepad2.b)
                    robot.carousel.setPower(-0.8);
                else
                    robot.carousel.setPower(0);

                if (gamepad2.dpad_up)
                    robot.intake.setPower(1);
                else if(gamepad2.dpad_down)
                    robot.intake.setPower(-1);
                else
                    robot.intake.setPower(0);

                if (gamepad2.y)
                    robot.slide.setPower(0.5);
                else if (gamepad2.a)
                    robot.slide.setPower(-0.5);
                else
                    robot.slide.setPower(0.1);

                if (gamepad2.dpad_right)
                    robot.bucket.setPower(0.5);
                else if (gamepad2.dpad_left)
                    robot.bucket.setPower(-0.5);
                else
                    robot.bucket.setPower(0);

                //robot.rotation.setPower(-gamepad2.right_stick_y);



                // Send telemetry message to signify robot running;
                //telemetry.addData("drive", "%.2f", y);
                //telemetry.addData("rotate", "%.2f", x);
                //telemetry.addData("strafe", "%.2f", rx);
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
            }
        }
    }
