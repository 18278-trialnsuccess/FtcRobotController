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
    public DcMotor motorIntake;
    public Servo servoShooter;
    public Servo servoGrab;
    public Servo servoRotate;


    public BNO055IMU imu;

    public double driveSpeed;

    private HardwareMap hardwareMap = null;

    public SpeedbotHardware(HardwareMap aHardwareMap, double speed) {

        hardwareMap = aHardwareMap;

        motorLF = hardwareMap.get(DcMotor.class, "LF");
        motorLB = hardwareMap.get(DcMotor.class, "LB");
        motorRF = hardwareMap.get(DcMotor.class, "RF");
        motorRB = hardwareMap.get(DcMotor.class, "RB");
        motorFlywheel = hardwareMap.get(DcMotor.class, "FM");
        motorIntake = hardwareMap.get(DcMotor.class, "IM");
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoShooter = hardwareMap.get(Servo.class, "SS");
        servoGrab = hardwareMap.get(Servo.class, "SG");
        servoRotate = hardwareMap.get(Servo.class, "SR");
        servoShooter.setDirection(Servo.Direction.FORWARD);
        servoGrab.setDirection(Servo.Direction.FORWARD);
        servoRotate.setDirection(Servo.Direction.FORWARD);


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

    public void strafe(double x, double y) {
        double lf;
        double lb;
        double rf;
        double rb;

        lf = y;
        lb = y;
        rf = y;
        rb = y;

        lf += x;
        lb -= x;
        rf -= x;
        rb += x;

        lf *= driveSpeed;
        lb *= driveSpeed;
        rf *= driveSpeed;
        rb *= driveSpeed;

        motorLF.setPower(lf);
        motorLB.setPower(lb);
        motorRF.setPower(rf);
        motorRB.setPower(rb);
    }
}
