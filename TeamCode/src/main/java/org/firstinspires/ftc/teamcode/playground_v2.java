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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a atwo wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="the_playground_v2", group="Iterative Opmode")
// @Disabled
public class playground_v2 extends OpMode
{
    private RobotHardware robot;
    private int clawState;
    private double beltSpeed;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap, 0.5);

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
        clawState = 0;
        moveClaw();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        setClawState();
        toggleBelt();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stopMove();
    }

    private void drive() {
        if (gamepad1.left_bumper) {
            robot.strafe(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        } else {
            robot.tank(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
    }

    private void setClawState() {
        if (gamepad2.dpad_left) {
            clawState = 2;
            moveClaw();
        } else if (gamepad2.dpad_right) {
            clawState = 3;
            moveClaw();
        }
    }

    private void moveClaw() {
        if (clawState == 1) { // starting config
            robot.servoCG.setPosition(1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) { }
            robot.servoCR.setPosition(0.2); // TODO: find positions
        } else if (clawState == 2) { // grabbing configuration
            robot.servoCG.setPosition(0.25);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) { }
            robot.servoCR.setPosition(0.9); // TODO: find positions


        } else if (clawState == 3) {
            robot.servoCG.setPosition(1);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) { }
            robot.servoCR.setPosition(0.40); // TODO: find positions


        }
    }

    private void toggleBelt() {
        if (gamepad2.left_bumper) {
            if (beltSpeed != 0) {
                beltSpeed = 0;
            }
            beltSpeed = -0.4;
        }
        if (gamepad2.right_bumper) {
            if (beltSpeed != 0) {
                beltSpeed = 0.4;
            }
            beltSpeed = 2;
        }
        robot.motorBelt.setPower(beltSpeed);
    }

    private void controlShooter() {
        if (gamepad2.right_trigger != 0) {
            robot.motorLauncher.setPower(0.94);
        } else {
            robot.motorLauncher.setPower(0.0);
        }
    }

}
