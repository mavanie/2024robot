package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "BasketAuton", preselectTeleOp = "DriverControl" )
public class BasketAuton extends LinearOpMode {

    private RobotCommon common;
    private ElapsedTime time = new ElapsedTime();
    private String step = "Basket Auton Ready";
    public static double T1 = 2;
    public static double T2 = 1.8;
    public static double T3 = 0.3;
    public static double T4 = 2;
    public static double T5 = 2.5;
    public static double T6 = 0.3;
    public static double T7 = 2;
    public static double T8 = 2;
    public static double T9 = 2.3;
    public static double T10  = 3.2;
    public static double T11 = 2;
    public static double T12 = 2;
    public static double VX1 = 500;
    public static double VY1 = 280;
    public static double ROT1 = -280;
    public static double VX2 = -500;
    public static double ROT3 = -800;
    public static double VY4 = -500;
    public static double A1 = 1.12;
    public static int S1 = 4300;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (opModeIsActive()) {
            step = "1, raise arm";
            common.moveArm(RobotCommon.ARM_DROP);
            wait(T1);
            step = "2, extend slide and move";
            common.moveSlides(RobotCommon.SLIDES_EXTENDED);
            common.setRobotSpeed(VX1, VY1, ROT1);
            wait(T2);
            common.setRobotSpeed(0, 0, 0);
            wait(T3);
            step = "3, drop sample";
            common.moveIntake(RobotCommon.IntakeOptions.OUT);
            wait(T4);
            step = "4, move back";
            common.setRobotSpeed(VX2, 0, 0);
            wait(T5);
            common.setRobotSpeed(0, 0, 0);
            wait(T6);
            step = "5, retract slides";
            common.moveIntake(RobotCommon.IntakeOptions.STOP);
            common.moveSlides(RobotCommon.SLIDES_RETRACTED);
            wait(T7);
            step = "6, move to end";
            common.setRobotSpeed(0, 0, ROT3);
            wait(T9);
            common.setRobotSpeed(0, VY4, 0);
            wait(T10);
            common.setRobotSpeed(0, 0, 0);
            step = "7, lower arm";
            common.moveArm(A1);
            wait(T11);
            step = "8, extend slide";
            common.moveSlides(S1);
            wait(T12);
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