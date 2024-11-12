package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class AltBasketAuton extends LinearOpMode {
    private RobotCommon common;
    private ElapsedTime opModeTime = new ElapsedTime();
    private ElapsedTime waitTime = new ElapsedTime();

    public static double CHAMBER_X = 15;

    public static double ARM_SPECIMEN = 0.7;
    public static double ARM_CLIP = 1.2;
    public static double ARM_SAMPLE = 1.65;
    public static double ARM_BASKET = 0.77;
    public static int SLIDE_SPECIMEN = 1000;
    public static int SLIDE_SAMPLE = 3700;
    public static int SLIDE_BASKET = 4800;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }
        common.run();
        common.moveArm(ARM_SPECIMEN);
//        wait(0.2);
//        common.moveSlides(SLIDE_SPECIMEN);

        common.driveX(1000, CHAMBER_X, this::run);

        waitSeconds(5);

    }


    private void waitSeconds(double t) throws InterruptedException{
        waitTime.reset();
        while (waitTime.seconds() < t){
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
        telemetry.addData("time", opModeTime.seconds());
        common.sendTelemetry(telemetry);
        telemetry.update();
    }
}
