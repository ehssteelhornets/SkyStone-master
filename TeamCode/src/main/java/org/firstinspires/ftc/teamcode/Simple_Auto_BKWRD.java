package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Simple_Auto_BKWRD", group ="Pushbot")
public class Simple_Auto_BKWRD extends LinearOpMode {
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
    static final double DIST_TO_FOUNDATION = 48;

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
        encoderDrive(10,  48,  48, 5.0);
        sleep(5000);
        //Drives forward to foundation
        encoderDrive(1.0,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(5000);
        encoderDrive(1.0,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(1.0, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move
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
                newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) ((leftInches - 8) * COUNTS_PER_INCH + 10);
                newRightTarget = robot.rightDrive.getCurrentPosition() + (int) ((rightInches - 8) * COUNTS_PER_INCH + 10);
                newLeftTarget2 = robot.leftDrive2.getCurrentPosition() + (int) ((leftInches - 8) * COUNTS_PER_INCH + 10);
                newRightTarget2 = robot.rightDrive2.getCurrentPosition() + (int) ((rightInches - 8) * COUNTS_PER_INCH + 10);
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
