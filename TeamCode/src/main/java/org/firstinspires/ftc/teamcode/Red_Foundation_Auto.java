package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
@Autonomous(name="Red_Foundation_Auto", group ="Pushbot")
public class Red_Foundation_Auto extends LinearOpMode {
    Pushbot_2019 robot = new Pushbot_2019();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark NeveRest 60 Motor Encoder
    static final double TICKS_PER_REV = 1680;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (TICKS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_CIRCUMFERENCE_INCHES);
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int test = 0;
    static final double DIST_TO_FOUNDATION = 33;

    @Override
    public void runOpMode() {
        //Initializes the robot
        robot.init(hardwareMap);
        //Reset our encoders
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        waitForStart();
        robot.tuckAwayClaw1.setPower(-.4);
        robot.tuckAwayClaw2.setPower(.8);
        //strafes left to change pivot point
        leftStrafe(1.0,8,2.0);
        sleep(800);
        //Drives forward to foundation
        encoderDrive(1.0, -34, -34, 2.0);
        //puts down foundation Hooks
        robot.foundHook1.setPosition(1.0);
        robot.foundHook2.setPosition(0.0);
        sleep(1000);
        //Pulls Foundation backward partially
        encoderDrive(1.0, 35, 35, 2.0);
        sleep(700);
        //Rotate CCW 90 deg
        //encoderDrive(1.0, -24, 24, 2.0);
        //sleep(700);
        //encoderDrive(1, (-DIST_TO_FOUNDATION), (-DIST_TO_FOUNDATION), 0.8);
        //encoderDrive(1, -DIST_TO_FOUNDATION, -DIST_TO_FOUNDATION, 0.2);
        //sleep(400);
        //Release Foundation
        robot.foundHook1.setPosition(0.0);
        robot.foundHook2.setPosition(1.0);
        sleep(500);
        //encoderDrive(0.5, (-WHEEL_CIRCUMFERENCE_INCHES), (WHEEL_CIRCUMFERENCE_INCHES), 2.5);
        //sleep(300);
        //encoderDrive(1, (DIST_TO_FOUNDATION), (DIST_TO_FOUNDATION), 1.2);
        rightStrafe(1.0,47,3.0);
        sleep(500);
        //FWD to line
        //encoderDrive(1, (30), (30), 5.0);
    }
//Encoder method
// Note: Reverse movement is obtained by setting a negative distance (not speed)

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);
                robot.leftDrive2.setTargetPosition(newLeftTarget2);
                robot.rightDrive2.setTargetPosition(newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftDrive.setPower(Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2", "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch(TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }

    public void leftStrafe(double speed,
                           double Inches,
                           double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(-newLeftTarget);
                robot.rightDrive.setTargetPosition(newRightTarget);
                robot.leftDrive2.setTargetPosition(newLeftTarget2);
                robot.rightDrive2.setTargetPosition(-newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                //robot.leftDrive.setPower(-Math.abs(speed));
                robot.leftDrive.setPower(Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                //robot.rightDrive2.setPower(-Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));

                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2" , "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch(TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }

    public void rightStrafe(double speed,
                            double Inches,
                            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTarget2;
        int newRightTarget2;
        // Ensure that the opmode is still active
        try {
            if (opModeIsActive()) {
                // Determine new target position, and pass to motor controller
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                robot.leftDrive.setTargetPosition(newLeftTarget);
                robot.rightDrive.setTargetPosition(-newRightTarget);
                robot.leftDrive2.setTargetPosition(-newLeftTarget2);
                robot.rightDrive2.setTargetPosition(newRightTarget2);
                // Turn On RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                runtime.reset();
                robot.leftDrive.setPower(Math.abs(speed));
                //robot.rightDrive.setPower(-Math.abs(speed));
                robot.rightDrive.setPower(Math.abs(speed));
                //robot.leftDrive2.setPower(-Math.abs(speed));
                robot.leftDrive2.setPower(Math.abs(speed));
                robot.rightDrive2.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                    // Display it for the driver.
                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                    telemetry.addData("Path2" , "Running at %7d :%7d",
                            robot.leftDrive.getCurrentPosition(),
                            robot.rightDrive.getCurrentPosition());
                    telemetry.addData("Back wheels", newLeftTarget2 + "" + newRightTarget2);
                    telemetry.update();
                }

                // Stop all motion
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive2.setPower(0);
                robot.rightDrive2.setPower(0);
                // Turn off RUN_TO_POSITION
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        } catch(TargetPositionNotSetException e) {
            telemetry.addData("Mission Failed", "We'll get 'em next time: " + test);
            telemetry.update();
        }
    }
}
