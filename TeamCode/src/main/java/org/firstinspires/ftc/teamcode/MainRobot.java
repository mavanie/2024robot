package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp(name = "Main Robot")
public class MainRobot extends LinearOpMode {
    private RobotCommon common;
    // Drive
    public static int FAST_FORWARD_FACTOR = 1400;
    public static int SLOW_FORWARD_FACTOR = 1000;
    public static int FAST_SIDEWAYS_FACTOR = 1200;
    public static int SLOW_SIDEWAYS_FACTOR = 720;
    public static int FAST_ROTATION_FACTOR = 1300;
    public static int SLOW_ROTATION_FACTOR = 900;
    // Slides
    public static double SLIDES_CONTROLS = -100;
    // Arms
    public static double ARM_CONTROLS = 0.01;

    // Gamepads
    public Gamepad g1;
    public Gamepad g2;


    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                controls();
                common.run();
                sendTelemetry();
            }
        }

    }
    private void initialize() {
        common = new RobotCommon(hardwareMap);
        common.initialize();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Gamepad
        g1 = new Gamepad();
        g2 = new Gamepad();
    }

    private void controls() {
        g1.copy(gamepad1);
        g2.copy(gamepad2);

        // Drive
        boolean movingFastSideways = g1.a && Math.abs(g1.left_stick_x) > Math.abs(g1.left_stick_y);
        boolean movingFastForward = g1.a && Math.abs(g1.left_stick_y) > Math.abs(g1.left_stick_x);
        double vx = movingFastSideways ? 0 : -square(g1.left_stick_y) * (g1.a ? FAST_FORWARD_FACTOR : SLOW_FORWARD_FACTOR);
        double vy = movingFastForward ? 0 : square(g1.left_stick_x) * (g1.a ? FAST_SIDEWAYS_FACTOR : SLOW_SIDEWAYS_FACTOR);
        double rot = square(g1.right_trigger - g1.left_trigger) * (g1.a ? FAST_ROTATION_FACTOR : SLOW_ROTATION_FACTOR);
        common.setRobotSpeed(vx, vy, rot);

        // Slides
        if (g2.b) {
            common.moveSlides(RobotCommon.SLIDES_EXTENDED);
        }
        if (g2.a) {
            common.moveSlides(RobotCommon.SLIDES_RETRACTED);
        }
        if (Math.abs(g2.right_stick_y) > 0.1){
            int pos = common.getSlideTargetPosition() + (int)Math.round(square(g2.right_stick_y) * SLIDES_CONTROLS);
            if (pos > RobotCommon.SLIDES_EXTENDED){
                pos = RobotCommon.SLIDES_EXTENDED;
            }
            if (pos < RobotCommon.SLIDES_RETRACTED){
                pos = RobotCommon.SLIDES_RETRACTED;
            }
            common.moveSlides(pos);
        }

        // Arm
        if (g2.dpad_up) {
            common.moveArm(RobotCommon.ARM_DROP);
        }
        if (g2.dpad_down) {
            common.moveArm(RobotCommon.ARM_GROUND);
        }
        if (g2.dpad_left) {
            common.moveArm(RobotCommon.ARM_HORIZONTAL);
        }
        if (Math.abs(g2.left_stick_y) > 0.1){
            double pos = common.getArmTargetPosition() + square(g2.left_stick_y) * ARM_CONTROLS;
            if (pos > RobotCommon.ARM_MAX){
                pos = RobotCommon.ARM_MAX;
            }
            if (pos < RobotCommon.ARM_MIN){
                pos = RobotCommon.ARM_MIN;
            }
            common.moveArm(pos);
        }
    }

    private void sendTelemetry(){
        common.sendTelemetry(telemetry);
        telemetry.update();
    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}




