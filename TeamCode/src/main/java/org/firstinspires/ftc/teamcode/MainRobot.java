package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Main Robot")
public class MainRobot extends LinearOpMode {
    private RobotCommon common;
    // Drive
    private double rot;
    private double vx;
    private double vy;

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
//        common.initialize();
//        dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//
//        // Gamepad
//        g1 = new Gamepad();
//        g2 = new Gamepad();
    }


    private void controls() {

    }
    private void sendTelemetry() {
    }


}




