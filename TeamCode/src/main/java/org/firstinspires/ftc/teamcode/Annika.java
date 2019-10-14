package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Annika
{
    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    //Defines the motors for the wheels
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    //Defines hashmap to get servo indexes
    public static final HashMap<String, Integer> ServoIndexes = new HashMap<String, Integer>();
        static{
            ServoIndexes.put("groundLock", 0);
            ServoIndexes.put("wrist", 1);
            ServoIndexes.put("finger", 2);
        }

    //Defines the wheel power
    private double[] wheelPower;

    //Defines the servo positions ([servo index (groundLock, wrist, finger)], [position (open/up, closed/down)]
    private static final double[][] SERVO_POSITIONS = {{90, 0},{90, 0}, {90, 0}};

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
        //Define the arrays
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
        wrist.setDirection((Servo.Direction.FORWARD));
        finger.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * Sets the forward speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for forward, negative for backward)
     */
    public void setForwardSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = spd;
        }
    }

    /**
     * Sets the rotational speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for right, negative for left)
     */
    public void setTurnSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            if(i % 2 == 0)
                wheelPower[i] = spd;
            else
                wheelPower[i] = -spd;
        }
    }

    /**
     * Sets the rotational speed of the robot
     *
     * @param spd the speed the motors will be set to (positive for right, negative for left)
     */
    public void setStrafeSpeed(double spd)
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            if(i % 3 == 0)
                wheelPower[i] = spd;
            else
                wheelPower[i] = -spd;
        }
    }

    //Sets the motors to the current values of wheelPower
    public void move()
    {
        leftFront.setPower(wheelPower[0]);
        rightFront.setPower(wheelPower[1]);
        leftRear.setPower(wheelPower[2]);
        rightRear.setPower(wheelPower[3]);
    }

    //Sets the power of the wheels to four separate values. Used for testing motors
    public void testWheels(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower)
    {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    /**
     *
     * @param index = the index of the applied servo (0 = groundLock, 1 = wrist, 2 = finger)
     * @param isOpen = whether or not the servo is in the "open" position
     */
    public void setServo (int index, boolean isOpen)
    {
        if(isOpen)
            finger.setPosition(SERVO_POSITIONS[index][0]);
        else
            finger.setPosition(SERVO_POSITIONS[index][1]);
    }
}