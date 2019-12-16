/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Encoder Test", group="Iterative Opmode")
@Disabled

//Encoders not working as intended, suspending code until functional
public class EncoderTest extends OpMode
{
    final static int DISTANCE = 300;
    // Declare runtime.
    private ElapsedTime runtime = new ElapsedTime();
    Annika jack = new Annika();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialization process start");
        //Define robot
        jack.init(hardwareMap);

        //Zero out motors and servos
        jack.setForwardSpeed(0);
        jack.move();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        /*
        Wheel Encoder Test
        Motor input order: left front, right front, left rear, right rear

        Directions: d-Pad (up = forward, down = backward)
        Wheel controls: X=Left front, Y=Right Front, B=Right Rear, A=Left Rear
         */

        boolean[][] buttonOutputs = {{gamepad1.dpad_up, gamepad1.dpad_down}, {gamepad1.x, gamepad1.y, gamepad1.a, gamepad1.b}};

        for(int d = 0; d < buttonOutputs[0].length; d++)
        {
            for(int f = 0; f < buttonOutputs[1].length; f++)
            {
                if(buttonOutputs[0][d] && buttonOutputs[1][f])
                {
                    if(d == 0)
                    {
                        jack.testEncoders(f, (int) (jack.getMotorPosition(f) + DISTANCE));
                    }
                    else
                    {
                        jack.testEncoders(f, (int) (jack.getMotorPosition(f) - DISTANCE));
                    }
                }
            }
        }


        // Show the elapsed game time and wheel position.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", jack.getMotorPosition(0), jack.getMotorPosition(1));
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", jack.getMotorPosition(2), jack.getMotorPosition(3));

        telemetry.addData("Up", gamepad1.dpad_up);
        telemetry.addData("Down", gamepad1.dpad_down);

        telemetry.addData("X", gamepad1.x);
        telemetry.addData("Y", gamepad1.y);
        telemetry.addData("A", gamepad1.a);
        telemetry.addData("B", gamepad1.b);
        //telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
