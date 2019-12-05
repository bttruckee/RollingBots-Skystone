package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Annika
{
    //Speed of the arm to lock its position
    private static final double LOCKED_SPEED = 1;
    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    /*Defines the motors for the wheels
    0 = leftFront
    1 = rightFront
    2 =leftRear
    3 = rightRear*/
    private DcMotor[] wheelMotors;

    //Defines the wheel power array
    private double[] wheelPower;

    //Defines hashmap to get servo indexes
    public static final HashMap<String, Integer> ServoIndexes = new HashMap<String, Integer>();
        static{
            ServoIndexes.put("groundLock", 0);
            ServoIndexes.put("wrist", 1);
            ServoIndexes.put("finger", 2);
        }

    //Defines the servo positions ([servo index (groundLock, wrist, finger)], [position (open/up, closed/down)]
    private static final double[][] SERVO_POSITIONS = {{1.0, 0.5}, {1.0, 0.0}, {0.0, 0.5}};

    //Defines the motors and servos for the arm
    private DcMotor arm;

    //An array of the servos
    private Servo[] servos;

    private Servo wrist;
    private Servo finger;

    //Defines the lock to pull the build site
    private Servo groundLock;

    //Whether or not the arm is locked in position
    private boolean armLocked;

    public Annika()
    { armLocked = false; }

    public void init(HardwareMap hwMap)
    {
        //Define the arrays
        wheelMotors = new DcMotor[4];
        wheelPower = new double[4];
        servos = new Servo[3];

        //Set hardwaremap to paramater
        this.hwMap = hwMap;

        //Set motor/servo variables to motors/servos in hwMap
        wheelMotors[0] = hwMap.get(DcMotor.class, "left_front");
        wheelMotors[1] = hwMap.get(DcMotor.class, "right_front");
        wheelMotors[2] = hwMap.get(DcMotor.class, "left_rear");
        wheelMotors[3] = hwMap.get(DcMotor.class, "right_rear");

        arm = hwMap.get(DcMotor.class, "arm");

        servos[Annika.ServoIndexes.get("groundLock")] = hwMap.get(Servo.class, "ground_lock");

        servos[Annika.ServoIndexes.get("wrist")] = hwMap.get(Servo.class, "wrist");
        servos[Annika.ServoIndexes.get("finger")] = hwMap.get(Servo.class, "finger");

        //Set motor directions
        wheelMotors[0].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[1].setDirection(DcMotor.Direction.FORWARD);
        wheelMotors[2].setDirection(DcMotor.Direction.REVERSE);
        wheelMotors[3].setDirection(DcMotor.Direction.FORWARD);

        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*for(DcMotor motor: wheelMotors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }*/

        servos[Annika.ServoIndexes.get("groundLock")].setDirection(Servo.Direction.FORWARD);

        servos[Annika.ServoIndexes.get("wrist")].setDirection((Servo.Direction.FORWARD));
        servos[Annika.ServoIndexes.get("finger")].setDirection(Servo.Direction.FORWARD);

        wheelMotors[0].setPower(0);
        wheelMotors[1].setPower(0);
        wheelMotors[2].setPower(0);
        wheelMotors[3].setPower(0);

        servos[Annika.ServoIndexes.get("groundLock")].setPosition(SERVO_POSITIONS[Annika.ServoIndexes.get("groundLock")][0]);

        servos[Annika.ServoIndexes.get("wrist")].setPosition(SERVO_POSITIONS[Annika.ServoIndexes.get("wrist")][0]);
        servos[Annika.ServoIndexes.get("finger")].setPosition(SERVO_POSITIONS[Annika.ServoIndexes.get("finger")][0]);
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
        for(int i = 0; i < wheelMotors.length; i++)
        {
            /*if(wheelPower[i] != 0)
            {
                lockMotor(wheelMotors[i],false);
                wheelMotors[i].setPower(wheelPower[i]);
            }
            else
            {
                lockMotor(wheelMotors[i],true);
                wheelMotors[i].setPower(LOCKED_SPEED);
            }*/
            wheelMotors[i].setPower(wheelPower[i]);
        }
    }

    //Sets the power of the wheels to four separate values. Used for testing motors
    public void testWheels(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower)
    {
        wheelMotors[0].setPower(leftFrontPower);
        wheelMotors[1].setPower(rightFrontPower);
        wheelMotors[2].setPower(leftRearPower);
        wheelMotors[3].setPower(rightRearPower);
    }

    /**
     * Moves the arm up and down
     * Keeps the arm in place if the speed is 0
     *
     * @param spd Speed the arm moves
     * @return the position of the arm
     */
    public void moveArm(double spd)
    {
        if(spd != 0)
        {
            lockMotor(arm,false);
            arm.setPower(spd);
        }
        else
        {
            lockMotor(arm,true);
            arm.setPower(LOCKED_SPEED);
        }
    }

    /**
     * Activates/Deactivates the arm of the given motor
     *
     * @param motor the motor being called
     * @param toLocked
     */
    private void lockMotor(DcMotor motor, boolean toLocked)
    {
        if(armLocked != toLocked) {
            armLocked = toLocked;

            if (toLocked) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setTargetPosition(motor.getCurrentPosition());
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    public double getServoPosition(int servo)
    {
        return servos[servo].getPosition();
    }

    /**
     *
     * @param index = the index of the applied servo (0 = groundLock, 1 = wrist, 2 = finger)
     * @param isOpen = whether or not the servo is in the "open" position
     */
    public void setServo (int index, boolean isOpen)
    {
        if(isOpen)
            servos[index].setPosition(SERVO_POSITIONS[index][0]);
        else
            servos[index].setPosition(SERVO_POSITIONS[index][1]);
    }
}