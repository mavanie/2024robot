package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "BetterSpecimenAuton", preselectTeleOp = "DriverControl")
public class BetterSpecimenAuton extends LinearOpMode {
    private RobotCommon common;
    public static Pose2d START = new Pose2d(9, -9, 0);
    public static double START_Y = -9;
    public static double CHAMBER_X = 35;
    public static double CHAMBER_Y = START_Y;
    public static double BACK_X = 20;
    public static double BACK_Y = START_Y;
    public static double ARM_SPECIMEN = 0.7;
    public static int SLIDE_SPECIMEN = 1000;
    public static double ARM_CLIP = 1.2;
    public static double T_CLIP = 1;
    public static double VX3 = -100;
    public static double VY3 = 500;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Pose2d initialPose = START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder moveToChamber = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(CHAMBER_X, CHAMBER_Y));
//        .splineTo(new Vector2d(36, 12), Math.toRadians(90));

//        TrajectoryActionBuilder moveBack = moveToChamber.fresh()
//                .waitSeconds(T_CLIP)
//                .strafeTo(new Vector2d(BACK_X, BACK_Y));

        waitForStart();
        if (opModeIsActive()) {
            runBlocking(new SequentialAction(
//                common.doMoveArm(ARM_SPECIMEN),
                moveToChamber.build()
//                common.doMoveSlides(SLIDE_SPECIMEN)
            ));

            TrajectoryActionBuilder moveBack2 = drive.actionBuilder(drive.pose)
                    .waitSeconds(T_CLIP)
                    .strafeTo(new Vector2d(BACK_X, BACK_Y));

            runBlocking(
                new ParallelAction(
                    common.doMoveArm(ARM_CLIP),
                    moveBack2.build()
                )
            );
        }
    }

    private void initialize(){
        common = new RobotCommon(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common.initialize();
        sendTelemetry();
    }

    public void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        // apply our coordinate system
        previewCanvas.setRotation(Math.toRadians(-90)).setTranslation(0, 72);
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

            running = action.run(packet);

            dash.sendTelemetryPacket(packet);
            common.runAuton();
            sendTelemetry();
        }
    }

    private void sendTelemetry(){
        common.sendTelemetry(telemetry);
        telemetry.update();
    }
}

// move the arm, extend slide, drop with intake, retract slide, move arm.