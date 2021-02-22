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

import org.firstinspires.ftc.robotcore.internal.network.InvalidNetworkSettingException;

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

@TeleOp(name="the_speedground", group="Iterative Opmode")
// @Disabled
public class the_speedground extends OpMode
{
    // Declare OpMode members.
    private SpeedbotHardware robot;
    private boolean intakestate;
    private long unlock;
    static final double     SERVO_PASSIVE = 0.3;
    static final double     SERVO_SHOOT = 0.0;
    static final double     FLYWHEEL_SPEED = 0.92;
    static final double     FLYWHEEL_POWERSHOT_SPEED = 0.82;
    static final double     INTAKE_SPEED = 1;
    static boolean clawUp = false;
    static boolean clawDown = true;
    static boolean clawOpen = true;
    static boolean clawClosed = false;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot = new SpeedbotHardware(hardwareMap, 0.8);
        robot.servoGrab.setPosition(0.8);
        robot.servoRotate.setPosition(0.35);
        unlock = System.nanoTime();
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
        robot.servoShooter.setPosition(0.25);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        drive();
        flywheel();
        servo();
        intakeToggle();
        clawGrab();
        clawRotate();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    private void drive() {
        if (gamepad1.left_bumper) {
            robot.strafe(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        } else {
            robot.tank(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        }
    }


    private void flywheel() {
        if (gamepad1.right_trigger != 0) {
            robot.motorFlywheel.setPower(-FLYWHEEL_SPEED);
        } else if (gamepad1.left_trigger != 0) {
            robot.motorFlywheel.setPower(-FLYWHEEL_POWERSHOT_SPEED);
        }
        else {
            robot.motorFlywheel.setPower(0);
        }
    }

    private void servo() {
        if (gamepad1.a) {

            robot.servoShooter.setPosition(SERVO_SHOOT);
            robot.sleep(200);
            robot.servoShooter.setPosition(SERVO_PASSIVE);
        }
    }

    private void intakeToggle() {
        if (gamepad1.right_bumper) {
            intakestate = !intakestate;
            intake();
            robot.sleep(300);
            telemetry.addData("power- right bumper", robot.motorIntake.getPower());
        }

    }

    private void intake() {
        if (intakestate) {
            robot.motorIntake.setPower(INTAKE_SPEED);
        } else {
            robot.motorIntake.setPower(0.0);
        }
    }

    /*private void clawGrab() {
        if (gamepad1.b && System.nanoTime() > unlock) {
            long unlock = System.nanoTime() + 100;
            if (robot.servoGrab.getPosition() == 0.8) {
                robot.servoGrab.setPosition(0.3);
            } else {
                robot.servoGrab.setPosition(0.8);
            }

        }
    }*/

    private void clawGrab() {
        if (gamepad1.b && clawOpen == true) {
            robot.servoGrab.setPosition(0.3);
            clawClosed = true;
            clawOpen = false;

        }else if(gamepad1.b && clawClosed == true){
            robot.servoGrab.setPosition(0.8);
            clawClosed = false;
            clawOpen = true;
        }
    }

    /*private void clawRotate() {
        if (gamepad1.y && System.nanoTime() > unlock) {
            long unlock = System.nanoTime() + 100;
            if (robot.servoRotate.getPosition() == 0.35) {
                robot.servoRotate.setPosition(0.7);
            } else {
                robot.servoRotate.setPosition(0.35);
            }
            robot.sleep(100);
        }
    }*/
    private void clawRotate() {
        if (gamepad1.y && clawUp == true) {
                //Get the claw down
            robot.servoRotate.setPosition(0.35);
           clawUp = false;
          clawDown = true;
        } else if(gamepad1.y && clawDown == true){
            robot.servoRotate.setPosition(0.7);
            clawDown = false;
            clawUp = true;
        }
    }

}
