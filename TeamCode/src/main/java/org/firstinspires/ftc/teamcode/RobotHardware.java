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


public class RobotHardware {


    public DcMotor motorLF;
    public DcMotor motorLB;
    public DcMotor motorRF;
    public DcMotor motorRB;


    public DcMotor motorBelt;
    public DcMotor motorLauncher;

    public Servo servoIntake;
    public Servo servoCR;
    public Servo servoCG;

    public BNO055IMU imu;

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

    public RobotHardware(HardwareMap aHardwareMap, double speed) {
        hardwareMap = aHardwareMap;

        motorLF = hardwareMap.get(DcMotor.class, "LF");
        motorLB = hardwareMap.get(DcMotor.class, "LB");
        motorRF = hardwareMap.get(DcMotor.class, "RF");
        motorRB = hardwareMap.get(DcMotor.class, "RB");
        motorLF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLB.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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
        motorLF.setPower(0);
        motorLB.setPower(0);
        motorRF.setPower(0);
        motorRB.setPower(0);
    }

    public void initializeIMU() {
        //------------------------------------------------------------
        // IMU - BNO055
        // Set up the parameters with which we will use our IMU.
        // + 9 degrees of freedom
        // + use of calibration file (see calibration program)
        //------------------------------------------------------------

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitImuCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.mode                = BNO055IMU.SensorMode.NDOF;

        parameters.accelerationIntegrationAlgorithm = null;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public org.firstinspires.ftc.robotcore.external.navigation.Orientation angularOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }


    public void resetDriveEncoders() {
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}