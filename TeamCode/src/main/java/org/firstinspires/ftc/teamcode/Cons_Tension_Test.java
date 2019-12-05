package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
@Autonomous(name="Cons_Tension_Test", group ="Pushbot")
public class Cons_Tension_Test extends LinearOpMode {
    Pushbot_2019 robot = new Pushbot_2019();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    // eg: AndyMark Orbital 20 Motor Encoder


    @Override
    public void runOpMode() {
        robot.vertExt.setPower(0.01);

    }
    }