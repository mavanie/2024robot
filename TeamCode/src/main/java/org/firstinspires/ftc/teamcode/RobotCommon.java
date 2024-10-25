package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

@Config

public class RobotCommon {
    private final HardwareMap hardwareMap;

    // Hook
    private Servo hook;
    public static double HOOK_EXTENDED = 1;
    public static double HOOK_RETRACTED = 0;

    // Wheels
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private double vx;
    private double vy;
    private double rot;

    private double frontLeftTarget;
    private double backLeftTarget;
    private double frontRightTarget;
    private double backRightTarget;

    private IMU imu;
    private double yaw;
    private AbsoluteGyro gyro;
    private double absoluteYaw;

    // Arm
    private DcMotorEx arm;
    private AnalogInput potentiometer;
    private double armPosition;
    private double armTargetPosition;
    private double armPower;
    public static double ARM_P = 100;
    public static double ARM_MIN = 0.684;
    public static double ARM_DROP = 0.9;
    public static double ARM_HORIZONTAL = 1.4;
    public static double ARM_GROUND = 2.094;
    public static double ARM_MAX = 2.41;

    // Slides
    private DcMotorEx slides;
    public static int SLIDE_VELOCITY = 5000;
    public static int SLIDES_EXTENDED = 5500;
    public static int SLIDES_RETRACTED = 0;

    // Intake
    private CRServo intakeLeft;
    private CRServo intakeRight;
    public enum IntakeOptions {
        STOP, IN, OUT
    }

    public RobotCommon(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void initialize() {
        // Bulk Read
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Init hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        imu = hardwareMap.get(IMU.class, "imuExpansion");
        hook = hardwareMap.get(Servo.class, "hook");

        // Config Imu
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    new Orientation(
                            AxesReference.INTRINSIC,
                            AxesOrder.ZYX,
                            AngleUnit.DEGREES,
                            180f,
                            0f,
                            60f,
                            0
                    )
                )
        );
        imu.initialize(imuParams);
        gyro = new AbsoluteGyro();

        // Config Motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        // Enable Encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Config slides
        slides.setTargetPosition(slides.getCurrentPosition());
        slides.setMode(RunMode.RUN_TO_POSITION);
        slides.setVelocity(SLIDE_VELOCITY);

        // Arm
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPosition = potentiometer.getVoltage();
        armTargetPosition = armPosition;
    }

    public void run() {
        runDrive();
        runArm();
    }

    // Wheels

    public void setRobotSpeed(double vx, double vy, double rot) {
    this.vx = vx;
    this.vy = vy;
    this.rot = rot;
    }


    private void runDrive() {
        yaw = imu.getRobotYawPitchRollAngles().getYaw();
        absoluteYaw = gyro.calculate(yaw);

        frontLeftTarget = vx + vy + rot;
        backLeftTarget = (vx - vy) + rot;
        frontRightTarget = (vx - vy) - rot;
        backRightTarget = (vx + vy) - rot;
        frontLeft.setVelocity(frontLeftTarget);
        backLeft.setVelocity(backLeftTarget);
        frontRight.setVelocity(frontRightTarget);
        backRight.setVelocity(backRightTarget);
    }

    public double getAbsoluteYaw() {
        return absoluteYaw;
    }
    public void resetYaw() {
        imu.resetYaw();
        // TODO: reset gyro yaw too
    }

    // Arm

    public void moveArm(double targetPosition){
        armTargetPosition = targetPosition;
    }

    public double getArmTargetPosition(){
        return armTargetPosition;
    }
    private void runArm(){
        armPosition = potentiometer.getVoltage();
        armPower = (armTargetPosition - armPosition) * ARM_P;
        arm.setPower(armPower/100);
    }

    // Hook
    public void moveHook(boolean extendHook){
        if (extendHook){
            hook.setPosition(HOOK_EXTENDED);
        } else {
            hook.setPosition(HOOK_RETRACTED);
        }
    }

    // Slide
    public void moveSlides(int targetPosition){
        slides.setTargetPosition(targetPosition);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setVelocity(SLIDE_VELOCITY);
    }
    public int getSlideTargetPosition() {
        return slides.getTargetPosition();
    }

    // Intake
    public void moveIntake (IntakeOptions intakeOption){
        if (intakeOption == IntakeOptions.OUT){
            intakeLeft.setPower(1);
            intakeRight.setPower(-1);
        } else if (intakeOption == IntakeOptions.IN){
            intakeLeft.setPower(-1);
            intakeRight.setPower(1);
        } else{
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }

    }
    public void sendTelemetry(Telemetry telemetry){
        telemetry.addData("Yaw", absoluteYaw);

//        telemetry.addData("frontLeftTarget", frontLeftTarget);
//        telemetry.addData("backLeftTarget", backLeftTarget);
//        telemetry.addData("frontRightTarget", frontRightTarget);
//        telemetry.addData("backRightTarget", backRightTarget);
//        telemetry.addData("frontLeftVelocity", frontLeft.getVelocity());
//        telemetry.addData("backLeftVelocity", backLeft.getVelocity());
//        telemetry.addData("frontRightVelocity", frontRight.getVelocity());
//        telemetry.addData("backRightVelocity", backRight.getVelocity());

        telemetry.addData("Slide Position", slides.getCurrentPosition());
        telemetry.addData("Slide Target Position", slides.getTargetPosition());
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Arm Target", armTargetPosition);
        telemetry.addData("Arm Power", armPower);
    }
}
