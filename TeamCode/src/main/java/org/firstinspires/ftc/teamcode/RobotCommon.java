package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.List;

public class RobotCommon {
    private final HardwareMap hardwareMap;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private double vx;
    private double vy;
    private double rot;
    private DcMotor arm;
    private DcMotor slides;
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
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        slides = hardwareMap.get(DcMotorEx.class,"slides");

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
    }

    public void run() {
        runDrive();
    }

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
    public void moveSlide(int targetPosition){
        slides.setTargetPosition(targetPosition);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public int getSlidePosition(){
        return slides.getCurrentPosition();
    }
    public int getSlideTargetPosition(){
        return slides.getTargetPosition();
    }
}
