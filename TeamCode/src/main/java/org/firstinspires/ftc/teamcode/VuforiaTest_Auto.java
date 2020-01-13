package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name="VuforiaTest_Auto", group ="Pushbot")

public class VuforiaTest_Auto extends LinearOpMode {
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
    static final double DIST_TO_FOUNDATION = 34;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AVBEWl3/////AAAAGbcR3zLzokcgvluieL+X7sUZchFxSNixRNcAWxR0bP9U+U43LsdX44KA/uRn41WeKGbwe4gTQcOdJve3abc+3SFCDWjM5NdDbjZkEutO2JcIigwn7jIU41jL6sXmCekHzJ7tW8F2B1JfISG6WP9KpbcD9F9BfMHnvBljUyT8nLP89/pqN0r8Zy2L5n9avC/LchzRCsMnvalZKZyYJkmlfNS8o4lKSOGzP2iEWx5a5J02jJiAwgPmsIjGKBWpUdwqB4fRlLLNxUMXKtRBIMEaLPn1+tdjMIIQX/fdf7q50MIWPdTdDdKJlbVHCiGrQa46ad5SA2+hFfsCglv8GW30Peuom9O5lGOWzjxSdcAv/H2W";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;   // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    @Override
    public void runOpMode() {
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
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        waitForStart();
        encoderDrive(1.0,-25,-25,2.0);// forward 25 inches
        sleep(700);
        leftStrafe(1.0,18,2.0);// strafe left 18 inches
        // initiate runtime
        // if recognize skystone target == true
            // forward
            // grab stone
            // backward same distance
            // strafe right

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.

        targetsSkyStone.activate();
        while (!isStopRequested()) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }

        // Disable Tracking when we are done;
        targetsSkyStone.deactivate();
    }
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
