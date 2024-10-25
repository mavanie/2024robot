package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "BasketAuton")
public class BasketAuton extends LinearOpMode {

    private RobotCommon common;
    private ElapsedTime time;
    private String step;
    private static double T1 = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        if (opModeIsActive()) {
            common.moveArm(RobotCommon.ARM_DROP);
            wait(T1);
            common.moveSlides(RobotCommon.SLIDES_EXTENDED);
            wait(T1);
            common.moveIntake(RobotCommon.IntakeOptions.OUT);
            wait(T1);
            common.moveSlides(RobotCommon.SLIDES_RETRACTED);
            wait(T1);
            common.moveArm(RobotCommon.ARM_HORIZONTAL);
            wait(T1);
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
        common.initialize();
    }
    private void run(){
        common.run();

    }
    private void sendTelemetry(){

    }
}

// move the arm, extend slide, drop with intake, retract slide, move arm.