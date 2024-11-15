package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Config

public class RobotCommon {
    private final HardwareMap hardwareMap;

    // Hook
    private Servo hook;
    public static double HOOK_EXTENDED = 0.35;
    public static double HOOK_RETRACTED = 1;

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
    public static double ARM_P_UP = 500;
    public static double ARM_P_DOWN = 200;
    public static double ARM_POWER_LIMIT = 10;
    public static double ARM_D = 500;
    private double armPPart = 0;
    private double armDPart = 0;
    public static double ARM_MIN = 0.684;
    public static double ARM_DROP = 0.7;
    public static double ARM_HORIZONTAL = 1.525;
    public static double ARM_GROUND = 1.921;
    public static double ARM_MAX = 1.95;
    private List<Double> oldArmPositions;
    private List<Double> speedFactors;
    private double armSpeed = 0;
    public static double armLimitedPower;
    public static double ARM_D_THERESHOLD = 0.05;

    // Slides
    private DcMotorEx slides;
    public static int SLIDE_VELOCITY = 5000;
    public static int SLIDES_EXTENDED = 4035;
    public static int SLIDES_RETRACTED = 0;
    public static int SLIDES_MAX = 4800;

    // Intake
    private CRServo intakeLeft;
    private CRServo intakeRight;
    public enum IntakeOptions {
        STOP, IN, OUT
    }
    private IntakeOptions currentIntakeOption = IntakeOptions.STOP;

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
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        // Arm
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armPosition = potentiometer.getVoltage();
        armTargetPosition = armPosition;
        speedFactors = Arrays.asList(-0.083, -0.059, -0.035, -0.012, 0.012, 0.035, 0.059, 0.083);
        oldArmPositions = new ArrayList<>(Collections.nCopies(speedFactors.size(), armPosition));
    }

    public void run() {
        runDrive();
        runArm();
    }

    public void runAuton() {
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

        oldArmPositions.add(armPosition);
        // Remove oldest item from list
        oldArmPositions.remove(0);
        armSpeed = 0;

        for (int i = 0; i < oldArmPositions.size(); i += 1) {
            armSpeed += oldArmPositions.get(i) * speedFactors.get(i);
        }
        double armError = armTargetPosition - armPosition;
        double p = armError > 0 ? ARM_P_DOWN : ARM_P_UP;
        armPPart = p * armError;
        armDPart = Math.abs(armError) >= ARM_D_THERESHOLD ? ARM_D * armSpeed : 0;
        armPower = Math.min(Math.max(armPPart - armDPart, -100), 100);
        armLimitedPower = Math.min(Math.max(armPower, armLimitedPower - ARM_POWER_LIMIT), armLimitedPower + ARM_POWER_LIMIT);
        arm.setPower(armLimitedPower / 100);
    }

    public void stopArm(){
        moveArm(armPosition);
    }

    public class MoveArmAction implements Action{
        private boolean firstRun = true;
        private final double targetPosition;
        public MoveArmAction(double targetPosition){
            this.targetPosition = targetPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun){
                moveArm(targetPosition);
                firstRun = false;
            }
            return Math.abs(targetPosition - armPosition) > 0.1;
        }
    }
    public Action doMoveArm(double targetPosition){
        return new MoveArmAction(targetPosition);
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
    public void stopSlides(){
        moveSlides(slides.getCurrentPosition());
    }
    public void resetSlides(){
        slides.setMode(RunMode.STOP_AND_RESET_ENCODER);
        moveSlides(0);
    }
    public class MoveSlidesAction implements Action{
        private boolean firstRun = true;
        private final int targetPosition;
        public MoveSlidesAction(int targetPosition){
            this.targetPosition = targetPosition;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (firstRun){
                moveSlides(targetPosition);
                firstRun = false;
            }
            return Math.abs(targetPosition - slides.getCurrentPosition()) > 10;
        }
    }
    public Action doMoveSlides(int targetPosition){
        return new MoveSlidesAction(targetPosition);
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
        currentIntakeOption = intakeOption;
    }

    public class MoveIntakeAction implements Action {
        private IntakeOptions intakeOption;
        public MoveIntakeAction(IntakeOptions intakeOption){
            this.intakeOption = intakeOption;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            moveIntake(intakeOption);
            return false;
        }
    }

    public Action doMoveIntake(IntakeOptions intakeOption){
        return new MoveIntakeAction(intakeOption);
    }

    public void sendTelemetry(Telemetry telemetry){
        telemetry.addData("Yaw", absoluteYaw);

//        telemetry.addData("frontLeftTarget", frontLeftTarget);
//        telemetry.addData("backLeftTarget", backLeftTarget);
//        telemetry.addData("frontRightTarget", frontRightTarget);
//        telemetry.addData("backRightTarget", backRightTarget);
        telemetry.addData("frontLeftVelocity", frontLeft.getVelocity());
        telemetry.addData("backLeftVelocity", backLeft.getVelocity());
        telemetry.addData("frontRightVelocity", frontRight.getVelocity());
        telemetry.addData("backRightVelocity", backRight.getVelocity());

        telemetry.addData("Slide Position", slides.getCurrentPosition());
        telemetry.addData("Slide Target Position", slides.getTargetPosition());
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Arm Target", armTargetPosition);
        telemetry.addData("Arm Power", armPower);
        telemetry.addData("Arm Speed", armSpeed);
        telemetry.addData("Arm D", armDPart);
        telemetry.addData("Arm P", armPPart);
        telemetry.addData("Arm Limited Power", armLimitedPower);
        telemetry.addData("Intake", currentIntakeOption.ordinal());
    }

    public void sendTelemetryAuton(TelemetryPacket packet){
//        packet.put("Yaw", absoluteYaw);

        packet.put("frontLeftVelocity", frontLeft.getVelocity());
        packet.put("backLeftVelocity", backLeft.getVelocity());
        packet.put("frontRightVelocity", frontRight.getVelocity());
        packet.put("backRightVelocity", backRight.getVelocity());

        packet.put("Slide Position", slides.getCurrentPosition());
        packet.put("Slide Target Position", slides.getTargetPosition());

        packet.put("Arm Position", armPosition);
        packet.put("Arm Target", armTargetPosition);
        packet.put("Arm Power", armPower);
        packet.put("Arm Speed", armSpeed);
        packet.put("Arm D", armDPart);
        packet.put("Arm P", armPPart);
        packet.put("Arm Limited Power", armLimitedPower);
        packet.put("Intake", currentIntakeOption.ordinal());
    }
}
