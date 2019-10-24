package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Pushbot_2019 {
    public DcMotor leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftDrive2   = null;
    public DcMotor  rightDrive2  = null;
    public DcMotor vertExt = null;
    public Servo foundHook1 = null;
    public Servo foundHook2 = null;
    public Servo rightClaw = null;
    public Servo leftClaw = null;
    public Servo tuckAwayClaw = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    /* Constructor */
    public Pushbot_2019(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap  ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //Define and Initialize servos
        foundHook1  = hwMap.get(Servo.class, "foundHook1");
        foundHook2  = hwMap.get(Servo.class, "foundHook2");
        leftClaw  = hwMap.get(Servo.class, "leftClaw");
        rightClaw  = hwMap.get(Servo.class, "rightClaw");
        tuckAwayClaw  = hwMap.get(Servo.class, "tuckAwayClaw");
        //set servo to starting position
        foundHook1.setPosition(1.0);
        foundHook2.setPosition(0.0);
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        tuckAwayClaw.setPosition(1.0);
        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive2 = hwMap.get(DcMotor.class, "left_drive2");
        rightDrive2 = hwMap.get(DcMotor.class, "right_drive2");
        vertExt = hwMap.get(DcMotor.class, "vertExt");
        //Set motor direction
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftDrive2.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        vertExt.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        rightDrive2.setPower(0);
        leftDrive2.setPower(0);
        vertExt.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
