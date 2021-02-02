package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class SpeedbotHardware {

    public DcMotor motorLF;
    public DcMotor motorLB;
    public DcMotor motorRF;
    public DcMotor motorRB;
    public DcMotor motorFlywheel;
    public Servo servoShooter;

    public BNO055IMU imu;

    public double driveSpeed;

    private HardwareMap hardwareMap = null;

    public SpeedbotHardware(HardwareMap aHardwareMap, double speed) {

        hardwareMap = aHardwareMap;

        motorLF = hardwareMap.get(DcMotor.class, "LF");
        motorLB = hardwareMap.get(DcMotor.class, "LB");
        motorRF = hardwareMap.get(DcMotor.class, "RF");
        motorRB = hardwareMap.get(DcMotor.class, "RB");
       //  motorFlywheel = hardwareMap.get(DcMotor.class, "FM");
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
       //  motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       //  motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoShooter = hardwareMap.get(Servo.class, "SS");
        servoShooter.setDirection(Servo.Direction.FORWARD);


        driveSpeed = speed;
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) { }
    }

    public void tank(double left, double right) {
        left *= driveSpeed;
        right *= driveSpeed;

        motorLF.setPower(left);
        motorLB.setPower(left);

        motorRF.setPower(right);
        motorRB.setPower(right);
    }
}