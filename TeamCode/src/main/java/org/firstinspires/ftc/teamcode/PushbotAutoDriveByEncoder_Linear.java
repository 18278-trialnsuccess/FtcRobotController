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

import android.telephony.emergency.EmergencyNumber;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
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

    /* Declare Computer Vision compnents */
    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;
    SkystoneDeterminationPipeline.RingPosition startPosition;

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     SERVO_PASSIVE = 0.3;
    static final double     SERVO_SHOOT = 0;
    static final double     FLYWHEEL_SPEED = 0.92;
    static final double     INTAKE_SPEED = 0.7;

    @Override
    public void runOpMode() {
        initAuto();
        tele("Start", startPosition);
        robot.servoRotate.setPosition(0.7);
        robot.servoShooter.setPosition(0.27);
        switch (startPosition) {
            case NONE:
                runStackNone();
                break;
            case ONE:
                runStackOne();
                break;
            case FOUR:
                runStackFour();
                break;
        }
//        robot.motorFlywheel.setPower(FLYWHEEL_SPEED);
//        encoderStrafe(0.53, 50, 50, 3);
//        shoot();
//        encoderStrafe(0.53, 7, 7, 7);
//        shoot();
//        encoderStrafe(0.53, 8, 8, 8);
//        shoot();
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void runStackNone() {
        encoderDrive(0.5, 59.5,60,10);
        robot.motorFlywheel.setPower(-FLYWHEEL_SPEED);
        sleep(1500);
        shoot();
        sleep(800);
        shoot();
        sleep(800);
        shoot();
        sleep(800);
        encoderDrive(0.5, 20, -20, 3);
        robot.motorFlywheel.setPower(0.0);
        encoderDrive(0.5, 15, 15, 3);
        robot.servoRotate.setPosition(0.35);
        robot.servoGrab.setPosition(0.8);
        sleep(1000);
        encoderDrive(0.5, -17, -17, 3);
        encoderDrive(0.5, 23, -23, 3);
        encoderDrive(0.5, 49.5, 49.5, 5);
        encoderDrive(0.1, 3, 3, 5);
        robot.servoGrab.setPosition(0.3);
        sleep(500);
        robot.servoRotate.setPosition(0.7);
        encoderDrive(0.5, -68, -68, 5);
        encoderDrive(0.5, -20,20,3);
        encoderDrive(0.5, 20,20,3);
        robot.servoRotate.setPosition(0.35);
        sleep(500);
        robot.servoGrab.setPosition(0.8);
    }
    private void runStackOne() {
        encoderDrive(0.5, 59.5,60,10);
        robot.motorFlywheel.setPower(-FLYWHEEL_SPEED);
        sleep(1500);
        shoot();
        sleep(800);
        shoot();
        sleep(1000);
        shoot();
        sleep(800);
        encoderDrive(0.5, 20,20,5);
        encoderDrive(0.5,10,-10,5);
        robot.motorFlywheel.setPower(0.0);
        robot.servoRotate.setPosition(0.35);
        robot.servoGrab.setPosition(0.8);

    }
    private void runStackFour() {
        encoderDrive(0.5, 59.5,60,10);
        robot.motorFlywheel.setPower(-FLYWHEEL_SPEED);
        sleep(1500);
        shoot();
        sleep(800);
        shoot();
        sleep(1000);
        shoot();
        sleep(800);
        robot.motorFlywheel.setPower(0.0);
        encoderDrive(0.5, 50,50,5);
        encoderDrive(0.5, 20,-20,5);
        encoderDrive(0.5, 20,20,5);
        robot.servoRotate.setPosition(0.35);
        robot.servoGrab.setPosition(0.8);
        sleep(1000);
        encoderDrive(0.5, -20,20,5);
        encoderDrive(0.5,-40,-40,5);
    }

    private void tele(String caption, Object value) {
        telemetry.addData(caption, value);
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

        robot.servoGrab.setPosition(0.3);
        robot.servoRotate.setPosition(1.0);
        waitForStart();
        initCV();

        // Wait for the game to start (driver presses PLAY)

    }

    private void initCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

        detectStarterStack();

    }

    private void shoot() {

        robot.servoShooter.setPosition(SERVO_SHOOT);
        robot.sleep(200);
        robot.servoShooter.setPosition(SERVO_PASSIVE);


    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            double rightOffset = 0.01;

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRF.getCurrentPosition() + (int)((rightInches) * COUNTS_PER_INCH);
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
            robot.motorRF.setPower(Math.abs(speed) + rightOffset);
            robot.motorRB.setPower(Math.abs(speed) + rightOffset);

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

    public void encoderStrafe(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLF.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motorRF.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motorLF.setTargetPosition(-newLeftTarget);
            robot.motorLB.setTargetPosition(newLeftTarget);
            robot.motorRF.setTargetPosition(newRightTarget);
            robot.motorRB.setTargetPosition(-newRightTarget);

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

    public void detectStarterStack() {
        for (int i = 0; i < 30; i++) {
            startPosition = pipeline.position;
            tele("Start", startPosition);
            sleep(50);
        }

    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(117 ,136);

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}
