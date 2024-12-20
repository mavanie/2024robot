package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "SpecimenAuton", preselectTeleOp = "DriverControl")
public class SpecimenAuton extends LinearOpMode {

    private RobotCommon common;
    private ElapsedTime time = new ElapsedTime();
    private String step = "Basket Auton Ready";
    public static double T1 = 2;
    public static double T2 = 2.4;
    public static double T3 = 0.3;
    public static double T4 = 1;
    public static double T5 = 1.5;
    public static double T6 = 5;
    public static double VX1 = 500;
    public static double VX2 = -500;
    public static double ARM_SPECIMEN = 0.7;
    public static int SLIDE_SPECIMEN = 1000;
    public static double ARM_CLIP = 1.2;
    public static double VX3 = -100;
    public static double VY3 = 500;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (opModeIsActive()) {
            step = "1, raise arm";
            common.moveArm(ARM_SPECIMEN);
            wait(T1);
            step = "2, extend slide and move";
            common.moveSlides(SLIDE_SPECIMEN);
            common.setRobotSpeed(VX1, 0, 0);
            wait(T2);
            common.setRobotSpeed(0, 0, 0);
            wait(T3);
            step = "3, clip specimen";
            common.moveArm(ARM_CLIP);
            wait(T4);
            step = "4, move back";
            common.setRobotSpeed(VX2, 0, 0);
            wait(T5);
            step = "5, go park";
            common.setRobotSpeed(VX3, VY3, 0);
            wait(T6);
            common.setRobotSpeed(0, 0, 0);
        }
    }
    private void wait(double t) throws InterruptedException{
        time.reset();
        while (time.seconds() < t){
            run();
        }
    }
    private void initialize(){
        common = new RobotCommon(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common.initialize();
        sendTelemetry();
    }
    private void run(){
        common.run();
        sendTelemetry();
    }
    private void sendTelemetry(){
        telemetry.addLine(step);
        telemetry.addData("time", time.seconds());
        common.sendTelemetry(telemetry);
        telemetry.update();
    }
}

// move the arm, extend slide, drop with intake, retract slide, move arm.