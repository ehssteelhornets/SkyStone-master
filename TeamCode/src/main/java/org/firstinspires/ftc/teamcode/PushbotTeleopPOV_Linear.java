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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
public class PushbotTeleopPOV_Linear extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static double right;
    static double left;
    static double right2;
    static double left2;
    static double foundHook = 1;
    static double tuckAway = 1;
    Pushbot_2019 robot = new Pushbot_2019();

    @Override
    public void runOpMode() {
        // Save reference to Hardware map
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        int mode = 1;
        telemetry.addData("opModeIsActive", opModeIsActive());
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.update();
            if (gamepad1.right_stick_button && mode == 1) {
                mode++;
                sleep(100);
            } else if (gamepad1.right_stick_button && mode == 2) {
                mode--;
                sleep(100);
            }
            //Tank Drive
            if (mode == 2) {
                telemetry.addData("Driving", "false");
                telemetry.update();
                right = gamepad1.right_stick_y;
                left = gamepad1.left_stick_y;
                boolean precisionMode = false;
                if (gamepad2.left_trigger != 0)
                    precisionMode = true;
                boolean reverseMode = false;
                if (gamepad2.right_trigger != 0)
                    reverseMode = true;
                drive(precisionMode, reverseMode);
            } else if (mode == 1) {
                //Mech Drive
                telemetry.addData("Driving", "true");
                telemetry.update();
                double r = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
                double robotAngle = Math.atan2(-gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;
                robot.leftDrive.setPower(v1);
                robot.rightDrive.setPower(v2);
                robot.leftDrive2.setPower(v3);
                robot.rightDrive2.setPower(v4);
            }


            //foundation hook toggle
            if (gamepad2.a && foundHook != 0) {
                foundHook--;
                sleep(300);
            } else if (gamepad2.a) {
                foundHook++;
                sleep(300);
            }

            if (foundHook != 0) {
                robot.foundHook1.setPosition(0.0);
                robot.foundHook2.setPosition(1.0);
            }
            if (foundHook == 0) {
                robot.foundHook1.setPosition(1.0);
                robot.foundHook2.setPosition(0.0);
            }


            //Controls linear actuator
            // Show the elapsed game time and wheel power.................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................................
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", right, left);
            telemetry.update();

            //linear actuator
            robot.vertExt.setPower(gamepad2.left_stick_y);
      /*
            if (gamepad2.dpad_up) {
                robot.vertExt.setPower(-1);
            } else if (gamepad2.dpad_down) {
                robot.vertExt.setPower(1);
            } else if (gamepad2.dpad_right) {
                robot.vertExt.setPower(0);
*/


            //claw grabber thingy
            if (gamepad1.right_bumper) {//Closed
                robot.rightClaw.setPosition(0.65);
                robot.leftClaw.setPosition(0.2);//Works PERFECTLY DALTON (DONT TOUCH)

            } else if (gamepad1.left_bumper) {//Opened
                //clawOffset = 0.0;
                //inversly proportional
                robot.rightClaw.setPosition(0.5);//WORKS PERFECTLY DALTON (DONT TOUCH)
                robot.leftClaw.setPosition(0.35);

            } else if (gamepad1.left_trigger != 0) {//Opened
            //clawOffset = 0.0;
            //inversly proportional
            robot.rightClaw.setPosition(0.3);//WORKS PERFECTLY DALTON (DONT TOUCH)
            robot.leftClaw.setPosition(0.55);
            }

            //tuckawayclaw toggle
            if (gamepad2.x && tuckAway != 0) {
                tuckAway--;
                sleep(300);
            } else if (gamepad2.x) {
                tuckAway++;
                sleep(300);
            }

            if (tuckAway == 0) {
                robot.tuckAwayClaw1.setPower(.8);
                robot.tuckAwayClaw2.setPower(.8);
            }
            if (tuckAway != 0) {
                robot.tuckAwayClaw1.setPower(-.4);
                robot.tuckAwayClaw2.setPower(-.4);


            }

        }
    }
    void drive (boolean precise, boolean reverse){
        double right_scaled = scaleMotor(right, precise);
        double left_scaled = scaleMotor(left, precise);

        if (reverse) {
            double temp = right_scaled;
            right_scaled = -left_scaled;
            left_scaled = -temp;
        }
        //Set power for motors
        robot.rightDrive.setPower(right_scaled);
        robot.rightDrive2.setPower(right_scaled);
        robot.leftDrive.setPower(left_scaled);
        robot.leftDrive2.setPower(left_scaled);
    }
    double scaleMotor ( double num, boolean precise){
        if (num == 0.0)
            return 0.0;
        //For precision mode
        double[] scaleArray = {0.5, 0.75, 1.0};
        double[] preciseArray = {0.1, 0.2, 0.3};
        // get the corresponding index for the scaleInput array.
        int index = (int) (num * (scaleArray.length - 1));
        index = Math.abs(index);
        double scaled;
        if (precise)
            scaled = preciseArray[index];
        else
            scaled = scaleArray[index];
        if (num < 0.0)
            scaled = -scaled;
        return scaled;
    }
}

