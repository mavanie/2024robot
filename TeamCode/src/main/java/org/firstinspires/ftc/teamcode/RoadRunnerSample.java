package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous
public class RoadRunnerSample extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotCommon common = new RobotCommon(hardwareMap);
        common.initialize();

        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(90)), 0)
                .strafeTo(new Vector2d(20, 20))
                .turn(Math.toRadians(180))
                    ;

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                tab1.build(),
                common.doMoveIntake(RobotCommon.IntakeOptions.OUT)
                )
                );
    }
}
