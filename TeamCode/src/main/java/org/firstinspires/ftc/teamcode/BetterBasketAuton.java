package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class BetterBasketAuton extends LinearOpMode {
    private RobotCommon common;
    private ElapsedTime opModeTime = new ElapsedTime();
    public static double START_X = -8.75;
    public static double START_Y = -62.75;
    public static double START_R = 90;
    public static double CHAMBER_Y = -38;
    public static double BACK_Y = -48;
    public static double SAMPLE1_X = -52;
    public static double SAMPLE1_Y = -48;
    public static double SAMPLE1_R = 70;
    public static double BASKET_R = -135;
    public static double ARM_SPECIMEN = 0.7;
    public static double ARM_CLIP = 1.2;
    public static double ARM_SAMPLE = 1.65;
    public static double ARM_BASKET = 0.77;
    public static int SLIDE_SPECIMEN = 1000;
    public static int SLIDE_SAMPLE = 3700;
    public static int SLIDE_BASKET = 4800;
    public static double T_START = 0.6;
    public static double T_CLIP = 1;
    public static double T_SAMPLE = 1;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = new Pose2d(START_X, START_Y, Math.toRadians(START_R));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .afterTime(0, common.doMoveArm(ARM_SPECIMEN))
            .afterTime(0.2, common.doMoveSlides(SLIDE_SPECIMEN))
            .waitSeconds(T_START)
            .strafeTo(new Vector2d(START_X, CHAMBER_Y))
            .stopAndAdd(new SequentialAction(
                new InstantAction(() -> common.moveArm(ARM_CLIP)) // don't wait for the arm to reach its target
            ))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(START_X, BACK_Y))
            .afterTime(0, new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveArm(ARM_SAMPLE)))
            .afterTime(0, common.doMoveArm(ARM_SAMPLE))
            .strafeToSplineHeading(new Vector2d(SAMPLE1_X, SAMPLE1_Y), Math.toRadians(SAMPLE1_R), (robotPose, _path, _disp) -> {
                if (robotPose.position.x.value() < SAMPLE1_X + 5) {
                    return 10.0;
                } else {
                    return 40.0;
                }
            })
            .stopAndAdd(new ParallelAction(
                common.doMoveIntake(RobotCommon.IntakeOptions.IN),
                common.doMoveSlides(SLIDE_SAMPLE)
            ))
            .waitSeconds(T_SAMPLE)
            .stopAndAdd(new ParallelAction(
                common.doMoveIntake(RobotCommon.IntakeOptions.STOP),
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED)
            ))
        .afterTime(0, new SequentialAction(
            common.doMoveArm(ARM_BASKET),
            common.doMoveSlides(SLIDE_BASKET)
        ))
            .turnTo(Math.toRadians(BASKET_R))
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(SLIDE_BASKET),
                common.doMoveIntake(RobotCommon.IntakeOptions.OUT)
            ))
            .waitSeconds(10);

        Action trajectoryAction = trajectory.build();
        preview(trajectoryAction);

        waitForStart();
        if (opModeIsActive()) {
            opModeTime.reset();
            runBlocking(trajectoryAction);
        }
    }

    private void initialize(){
        common = new RobotCommon(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common.initialize();
    }

    public void preview(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());

        packet.put("time", 0);
        packet.put("heading (deg)", START_R);
        packet.put("headingError (deg)", 0);
        packet.put("x", START_X);
        packet.put("xError", 0);
        packet.put("y", START_Y);
        packet.put("yError", 0);

        common.sendTelemetryAuton(packet);
        dash.sendTelemetryPacket(packet);
    }

    public void runBlocking(Action action) {
        FtcDashboard dash = FtcDashboard.getInstance();
        Canvas previewCanvas = new Canvas();
        action.preview(previewCanvas);

        boolean running = true;
        while (running && !Thread.currentThread().isInterrupted()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(previewCanvas.getOperations());
            packet.put("time", opModeTime.seconds());

            running = action.run(packet);

            common.runAuton();
            common.sendTelemetryAuton(packet);
            dash.sendTelemetryPacket(packet);
        }
    }

}