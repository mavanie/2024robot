package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@TeleOp(name = "Main Robot")
public class MainRobot extends LinearOpMode {
    private RobotCommon common;
    // Drive
    private double rot;
    private double vx;
    private double vy;
    int FAST_FORWARD_FACTOR = 1400;
    int SLOW_FORWARD_FACTOR = 1000;
    int FAST_SIDEWAYS_FACTOR = 1200;
    int SLOW_SIDEWAYS_FACTOR = 720;
    int FAST_ROTATION_FACTOR = 1300;
    int SLOW_ROTATION_FACTOR = 900;
    public static int SLIDES_EXTENDED = 1000;
    public static int SLIDES_RETRACTED = 0;

    // Gamepads
    public Gamepad g1;
    public Gamepad g2;


    @Override
    public void runOpMode() throws InterruptedException {
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

        telemetry = FtcDashboard.getInstance().getTelemetry();

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
        vx = movingFastSideways ? 0 : -square(g1.left_stick_y) * (g1.a ?  FAST_FORWARD_FACTOR : SLOW_FORWARD_FACTOR);
        vy = movingFastForward ? 0 : square(g1.left_stick_x) * (g1.a ? FAST_SIDEWAYS_FACTOR : SLOW_SIDEWAYS_FACTOR);
        rot = square(g1.right_trigger - g1.left_trigger) * (g1.a ? FAST_ROTATION_FACTOR : SLOW_ROTATION_FACTOR);
        common.setRobotSpeed(vx, vy, rot);

        // Slides
        if (gamepad2.a) {
            common.moveSlide(SLIDES_EXTENDED);
        }
        if (gamepad2.b) {
            common.moveSlide(SLIDES_RETRACTED);
        }
    }
    private void sendTelemetry() {
        telemetry.addData("Slide Position", common.getSlidePosition());
        telemetry.addData("Slide Target Position", common.getSlideTargetPosition());
        telemetry.update();
    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}




