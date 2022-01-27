/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop.teleop8648;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardwarepusbots.Frenzy8648HardwarePushbot;


/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="8648 Teleop Test", group="Pushbot")
@Disabled
public class Frenzy8648TeleopTest extends LinearOpMode {

    /* Declare OpMode members. */
    Frenzy8648HardwarePushbot robot           = new Frenzy8648HardwarePushbot();   // Use a Pushbot's hardware

    @Override
    public void runOpMode() {
       // double x1; //left;
        //double y1; //right;
      //  double pivot;

       // double drive;
        //double rotate;
        //double strafe;
        double power;
        double shoulder;
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, true);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            /*drive = -gamepad1.left_stick_x;
            strafe  =  gamepad1.left_stick_y;
            rotate = gamepad1.right_stick_x;
             */

            //left = -gamepad1.left_stick_y;
            //right = -gamepad1.right_stick_y;

            shoulder = gamepad2.right_stick_y;

            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;


            robot.leftFront.setPower(left);
            robot.rightFront.setPower(right);
            robot.leftBack.setPower(left);
            robot.rightBack.setPower(right);

            // Output the safe vales to the motor drives.
           // robot.leftFront.setPower(pivot + (-y1 + x1));
           // robot.rightFront.setPower(pivot + (-y1 + x1));
            //robot.leftBack.setPower(pivot + (-y1 + x1));
          //  robot.rightBack.setPower(pivot + (-y1 + x1));

            /*robot.leftFront.setPower(drive + rotate);
            robot.leftBack.setPower(drive + rotate);
            robot.rightFront.setPower(drive - rotate);
            robot.rightBack.setPower(drive - rotate);

             */
            //robot.armMotor.setPower(shoulder);


            //robot.e.setPower(robot.SHOOTER);
            //robot.w.setPosition((-gamepad2.left_stick_y / 2) + 0.5);

         /*   if (gamepad2.dpad_right) {
                //robot.e.setPosition(0.0);
                robot.w.setPosition(0.0);
                robot.t.setPosition(0.72);
                //_servo.setPosition(0.30);
                //r_servo.setPosition(0.94);
            }
            if (gamepad2.dpad_down) {
                //robot.e.setPosition(0.46);
                robot.w.setPosition(0.98);
                robot.t.setPosition(0.65);
                robot.l.setPosition(1.00);
                robot.r.setPosition(0.00);
            }
            if (gamepad2.dpad_left) {
                //robot.e.setPosition(0.36);
                robot.w.setPosition(0.84);
                robot.t.setPosition(0.65);
                robot.l.setPosition(1.00);
                robot.r.setPosition(0.00);
            }
            if (gamepad2.dpad_up) {
                //robot.e.setPosition(0.44);
                robot.w.setPosition(0.76);
                robot.t.setPosition(0.65);
                //s_servo.setPosition(0.50);
                robot.l.setPosition(1.00);
                robot.r.setPosition(0.00);
            }

            if(gamepad2.b)
                robot.r.setPosition(0.44);
            if (gamepad2.x)
                robot.r.setPosition(0.0);

            if(gamepad2.b)
                robot.l.setPosition(0.62);
            if (gamepad2.x)
                robot.l.setPosition(1);

            /*if (gamepad2.a)
                robot.e.setPosition(0);
            if (gamepad2.y)
                robot.e.setPosition(1);
             */

          /*  if(gamepad2.dpad_down)
                robot.w.setPosition(0);
            if(gamepad2.dpad_up)
                robot.w.setPosition(1);

            if (gamepad2.left_bumper)
                robot.s.setPower(1);
            else
                robot.s.setPower(0);

            /*if(gamepad2.dpad_down)
                robot.w.setPosition(1);
                robot.t.setPosition(0.72);
                //robot.e.setPosition(0.46);
                robot.s.setPosition(0.5);
            */
            // Send telemetry message to signify robot running;
            //telemetry.addData("right",  "%.2f", right);
            //telemetry.addData("left", "%.2f", left);
           // telemetry.addData("strafe",  "%.2f", strafe);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
