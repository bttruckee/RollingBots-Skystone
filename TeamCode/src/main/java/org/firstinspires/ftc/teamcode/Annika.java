package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Annika
{
    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    //Defines the motors for the wheels
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    //Defines the wheel power
    private double[] wheelPower;

    //Defines the motors and servos for the arm
    private DcMotor arm;
    private Servo wrist;
    private Servo finger;

    //Defines the lock to pull the build site
    private Servo groundLock;

    public Annika()
    {}

    public void init(HardwareMap hwMap)
    {
        //Set hardwaremap to paramater
        this.hwMap = hwMap;

        //Set motor/servo variables to motors/servos in hwMap
        leftFront = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftRear = hwMap.get(DcMotor.class, "left_rear");
        rightRear = hwMap.get(DcMotor.class, "right_rear");

        arm = hwMap.get(DcMotor.class, "arm");
        wrist = hwMap.get(Servo.class, "wrist");
        finger = hwMap.get(Servo.class, "finger");

        //Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.FORWARD);
    }
}