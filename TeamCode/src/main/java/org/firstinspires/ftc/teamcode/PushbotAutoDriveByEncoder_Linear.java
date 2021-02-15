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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto", group="Automated")

public class PushbotAutoDriveByEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    SpeedbotHardware robot;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        initAuto();
        encoderDrive(DRIVE_SPEED,  58,  55, 5.0);
        emptyBarrel();
        encoderDrive(DRIVE_SPEED,  9,  13, 5.0);



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void initAuto() {
        robot = new SpeedbotHardware(hardwareMap, 0.8);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.motorLF.getCurrentPosition(),
                          robot.motorRF.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    private void emptyBarrel() {
        robot.motorFlywheel.setPower(-SpeedbotHardware.SHOOTER_SPEED);
        robot.sleep(1500);

        robot.servoShooter.setPosition(0.8);
        robot.sleep(200);
        robot.servoShooter.setPosition(0.4);
        robot.sleep(1000);
        robot.servoShooter.setPosition(0.8);
        robot.sleep(200);
        robot.servoShooter.setPosition(0.4);
        robot.sleep(1000);
        encoderDrive(DRIVE_SPEED,  1,  0, 5.0);
        robot.motorFlywheel.setPower(-SpeedbotHardware.SHOOTER_SPEED + 0.01);
        robot.servoShooter.setPosition(0.8);
        robot.sleep(200);
        robot.servoShooter.setPosition(0.4);
        robot.sleep(1000);
        robot.motorFlywheel.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorLF.setTargetPosition(newLeftTarget);
            robot.motorLB.setTargetPosition(newLeftTarget);
            robot.motorRF.setTargetPosition(newRightTarget);
            robot.motorRB.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorLF.setPower(Math.abs(speed));
            robot.motorLB.setPower(Math.abs(speed));
            robot.motorRF.setPower(Math.abs(speed));
            robot.motorRB.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.motorLF.isBusy() && robot.motorRF.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.motorLF.getCurrentPosition(),
                                            robot.motorRF.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorLF.setPower(0);
            robot.motorLB.setPower(0);
            robot.motorRF.setPower(0);
            robot.motorRB.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
