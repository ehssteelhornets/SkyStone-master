package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
@Autonomous(name="Blue_Foundation_Auto", group ="Pushbot")
public class Blue_Foundation_Auto extends LinearOpMode {
    Pushbot_2019 robot = new Pushbot_2019();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: AndyMark Orbital 20 Motor Encoder from Video
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP AndyMark Orbital 20
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double WHEEL_CIRCUMFERENCE_INCHES = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;
    int test = 0;
    static final double DIST_TO_FOUNDATION = 12;

    @Override
    public void runOpMode() {
        robot.vertExt.setPower(0.1);

    }
    }