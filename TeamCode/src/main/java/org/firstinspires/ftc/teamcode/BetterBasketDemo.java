package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Config
@Autonomous(name = "BetterBasketDemo", preselectTeleOp = "DriverControl")
public class BetterBasketDemo extends LinearOpMode {
    private RobotCommon common;
    private ElapsedTime opModeTime = new ElapsedTime();
    public static double START_X = -34.5;
    public static double START_Y = -62.75;
    public static double START_R = 180;
    public static double BASKET1_X = -50.25;
    public static double BASKET1_Y = -54.25;
    public static double BASKET1_R = -135;
    public static double SAMPLE1_X = -46;
    public static double SAMPLE1_Y = -46;
    public static double SAMPLE1_R = 92;
    public static double SAMPLE_PUSH = 3;
    public static double SAMPLE2_X = -56;
    public static double SAMPLE2_Y = -46;
    public static double SAMPLE2_R = 93;
    public static double PARK_X = -40;
    public static double PARK_Y = -40;
    public static double PARK_R = 70;
    public static double PARK2_R = 40;
    public static double ARM_SAMPLE = 1.75;
    public static double ARM_BASKET = 0.77;
    public static double ARM_PARK = 1.1;
    public static int SLIDE_SAMPLE = 2500;
    public static int SLIDE_BASKET = 4800;
    public static double T_DROP = 1;
    public static double T_DROP1 = 1.5;
    public static double T_PARK = 1;
    public static double T_PARK2 = 1.5;
    public static double V_SLOW = 7;
    public static double R_SLOW = 1.8;
    public static double V_FAST = 60;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = new Pose2d(START_X, START_Y, Math.toRadians(START_R));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        VelConstraint slow = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(V_SLOW),
            new AngularVelConstraint(R_SLOW)
        ));

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .afterTime(0, common.doMoveArm(ARM_BASKET))
            .afterTime(0.2, common.doMoveSlides(SLIDE_BASKET))
            .waitSeconds(0.2)
            .strafeToSplineHeading(new Vector2d(BASKET1_X, BASKET1_Y), Math.toRadians(BASKET1_R), slow)
            .stopAndAdd(new SequentialAction(
//                common.doMoveSlides(SLIDE_BASKET),
                common.doMoveIntake(RobotCommon.IntakeOptions.OUT),
                new SleepAction(T_DROP1)
            ))
            .afterTime(0.3, common.doMoveSlides(RobotCommon.SLIDES_RETRACTED))
            .strafeToSplineHeading(new Vector2d(SAMPLE1_X, SAMPLE1_Y), Math.toRadians(SAMPLE1_R), slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveIntake(RobotCommon.IntakeOptions.IN),
                common.doMoveArm(ARM_SAMPLE),
                common.doMoveSlides(SLIDE_SAMPLE)
            ))
            .lineToY(SAMPLE1_Y + SAMPLE_PUSH, slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveIntake(RobotCommon.IntakeOptions.STOP)
            ))
            .afterTime(0, common.doMoveArm(ARM_BASKET))
            .afterTime(0.2, common.doMoveSlides(SLIDE_BASKET))
            .strafeToSplineHeading(new Vector2d(BASKET1_X, BASKET1_Y), Math.toRadians(BASKET1_R), slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(SLIDE_BASKET),
                common.doMoveIntake(RobotCommon.IntakeOptions.OUT),
                new SleepAction(T_DROP)
            ))
            .afterTime(0.3, common.doMoveSlides(RobotCommon.SLIDES_RETRACTED))
            .strafeToSplineHeading(new Vector2d(SAMPLE2_X, SAMPLE2_Y), Math.toRadians(SAMPLE2_R), slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveIntake(RobotCommon.IntakeOptions.IN),
                common.doMoveArm(ARM_SAMPLE),
                common.doMoveSlides(SLIDE_SAMPLE)
            ))
            .lineToY(SAMPLE2_Y + SAMPLE_PUSH, slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveIntake(RobotCommon.IntakeOptions.STOP)
            ))
            .afterTime(0, common.doMoveArm(ARM_BASKET))
            .afterTime(0.2, common.doMoveSlides(SLIDE_BASKET))
            .strafeToSplineHeading(new Vector2d(BASKET1_X, BASKET1_Y), Math.toRadians(BASKET1_R), slow)
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(SLIDE_BASKET),
                common.doMoveIntake(RobotCommon.IntakeOptions.OUT),
                new SleepAction(T_DROP)
            ))
            .stopAndAdd(common.doMoveIntake(RobotCommon.IntakeOptions.STOP))
            .strafeTo(new Vector2d(PARK_X, PARK_Y))
            .stopAndAdd(new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveArm(RobotCommon.ARM_GROUND)
            ));

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