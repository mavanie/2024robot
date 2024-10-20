package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config

public class RobotCommon {
    private final HardwareMap hardwareMap;

    // Wheels
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private double vx;
    private double vy;
    private double rot;

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
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setVelocity(SLIDE_VELOCITY);

        // Arm
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
        double wheel1;
        double wheel2;
        double wheel3;
        double wheel4;

        wheel1 = vx + vy + rot;
        wheel2 = (vx - vy) + rot;
        wheel3 = (vx - vy) - rot;
        wheel4 = (vx + vy) - rot;
        frontLeft.setVelocity(wheel1);
        backLeft.setVelocity(wheel2);
        frontRight.setVelocity(wheel3);
        backRight.setVelocity(wheel4);
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
        telemetry.addData("Slide Position", slides.getCurrentPosition());
        telemetry.addData("Slide Target Position", slides.getTargetPosition());
        telemetry.addData("Arm Position", armPosition);
        telemetry.addData("Arm Target", armTargetPosition);
        telemetry.addData("Arm Power", armPower);
    }
}
