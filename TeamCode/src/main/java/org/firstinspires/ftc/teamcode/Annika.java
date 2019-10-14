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
        //Set wheel power
        wheelPower = new double[4];

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

        groundLock = hwMap.get(Servo.class, "ground_lock");

        //Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.FORWARD);
        wrist.setDirection(Servo.Direction.FORWARD);
        finger.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sets the forward speeed of the robot
     *
     * @param spd = the forward speed of the robot (negative for reverse)
     */
    public void setForwardSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = spd;
        }
    }

    /**
     * Sets the rotational speed for the robot (negative for right)
     *
     * @param spd = speed for the wheels
     */
    public void setTurnSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            if(i % 2 == 0)
            {
                wheelPower[i] = spd;
            }
            else
            {
                wheelPower[i] = -spd;
            }
        }
    }

    /**
     * Sets the speed at which the robot strafes (negative for right)
     *
     * @param spd = speed for the wheels
     */
    public void setStrafeSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            if(i % 3 == 0)
            {
                wheelPower[i] = -spd;
            }
            else
            {
                wheelPower[i] = spd;
            }
        }
    }

    //Sets the respective powers of wheelPower to the four wheels
    public void runWheels()
    {
        leftFront.setPower(wheelPower[0]);
        rightFront.setPower(wheelpower[1]);
        leftRear.setPower(wheelpower[2]);
        rightRear.setPower(wheelPower[3]);
    }
}

