package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;


public class TestHardware {


    public DcMotor motorLF;
    public DcMotor motorLB;
    public DcMotor motorRF;
    public DcMotor motorRB;

    public ArrayList<DcMotor> drivetrain;




    public DcMotor motorBelt;
    public DcMotor motorLauncher;

    public Servo servoIntake;
    public Servo servoCR;
    public Servo servoCG;

    public double driveSpeed;

    public static final double SHOOTER_AUTO_TICKS = -1;
    public static final double SHOOTER_AUTO_DONE_SHOOT_TICKS = 800;

    public static final double INTAKE_ROTATE = 5000;

    public static final double WHEEL_DIAMETER = 4.0;
    public static final double DRIVE_MOTOR_TICKS_PER_ROTATION = 537.6;

    public static final double ANGLE_AUTO_UP_TICKS = 0.4;
    public static final double ANGLE_AUTO_DOWN_TICKS = -0.2;
    public static final double ANGLE_SHOOT_POS = -140;

    public static final double ARM_AUTO_DOWN_SPEED_SLOW = -0.2;
    public static final double ARM_AUTO_DOWN_SPEED_FAST = -1.0;
    public static final double ARM_AUTO_DOWN_SLOW_TICKS = 500;
    public static final double ARM_AUTO_UP_SPEED = 0.5;



    private HardwareMap hardwareMap = null;

    public TestHardware(HardwareMap aHardwareMap, double speed) {
        hardwareMap = aHardwareMap;
        drivetrain.add(motorLF);
        motorLF = hardwareMap.get(DcMotor.class, "LF");
        motorLB = hardwareMap.get(DcMotor.class, "LB");
        motorRF = hardwareMap.get(DcMotor.class, "RF");
        motorRB = hardwareMap.get(DcMotor.class, "RB");
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        drivetrain.add(motorLF);
        drivetrain.add(motorLB);
        drivetrain.add(motorRF);
        drivetrain.add(motorRB);
        for (DcMotor i : drivetrain) {
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }



        motorBelt = hardwareMap.get(DcMotor.class, "BM");
        motorBelt.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBelt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLauncher = hardwareMap.get(DcMotor.class, "LM");


        servoIntake = hardwareMap.get(Servo.class, "IS");
        servoCG = hardwareMap.get(Servo.class, "CG");
        servoCR = hardwareMap.get(Servo.class, "CR");

        driveSpeed = speed;
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

    public void stopMove() {
        for (DcMotor i : drivetrain) {
            i.setPower(0);
        }
    }




    public void resetDriveEncoders() {
        for (DcMotor i : drivetrain) {
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}