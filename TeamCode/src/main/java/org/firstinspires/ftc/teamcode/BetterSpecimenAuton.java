package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "BetterSpecimenAuton", preselectTeleOp = "DriverControl")
public class BetterSpecimenAuton extends LinearOpMode {
    private RobotCommon common;
    public static Pose2d START = new Pose2d(8.4, -63.25, Math.toRadians(90));
    public static double CHAMBER_X = 8.4;
    public static double CHAMBER_Y = -38;
    public static double BACK_X = 8.4;
    public static double BACK_Y = -48;
    public static double SIDE1_X = 34;
    public static double SIDE1_Y = -48;
    public static double FORWARD1_X = 36;
    public static double FORWARD1_Y = -12;
    public static double SAMPLE1_X = 45;
    public static double SAMPLE1_Y = -12;
    public static double PICKUP1_X = 45;
    public static double PICKUP1_Y = -57;
    public static double RETREAT_X = 48;
    public static double RETREAT_Y = -40;
    public static double CHAMBER2_X = 4.4;
    public static double CHAMBER2_Y = -38;
    public static double BACK2_X = 4.4;
    public static double BACK2_Y = -48;
    public static double PICKUP2_X = 48;
    public static double PICKUP2_Y = -57;
    public static double CHAMBER3_X = 0.4;
    public static double CHAMBER3_Y = -38;
    public static double BACK3_X = 0.4;
    public static double BACK3_Y = -48;
    public static double ARM_SPECIMEN = 0.7;
    public static int SLIDE_SPECIMEN = 1000;
    public static double ARM_CLIP = 1.2;
    public static double T_CLIP = 1;
    public static double T_INTAKE = 0.7;
    public static double T_HUMAN = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .afterTime(0, common.doMoveArm(ARM_SPECIMEN))
            .afterTime(0.5, common.doMoveSlides(SLIDE_SPECIMEN))
            .waitSeconds(0.2)
            .strafeTo(new Vector2d(CHAMBER_X, CHAMBER_Y))
            .stopAndAdd(new SequentialAction(
//                common.doMoveSlides(SLIDE_SPECIMEN), // ensure the slides finish moving before the arm starts
                new InstantAction(() -> common.moveArm(ARM_CLIP)) // don't wait for the arm to reach its target
            ))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(BACK_X, BACK_Y))
            .afterTime(0, new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveArm(RobotCommon.ARM_HORIZONTAL)))
            .strafeToLinearHeading(new Vector2d(SIDE1_X, SIDE1_Y), Math.toRadians(-89.99))
            .strafeTo(new Vector2d(FORWARD1_X, FORWARD1_Y))
            .strafeTo(new Vector2d(SAMPLE1_X, SAMPLE1_Y))
            .stopAndAdd(common.doMoveIntake(RobotCommon.IntakeOptions.IN))
            .strafeTo(new Vector2d(PICKUP1_X, PICKUP1_Y+3))
            .waitSeconds(T_HUMAN)
            .strafeTo(new Vector2d(PICKUP1_X, PICKUP1_Y))
            .waitSeconds(T_INTAKE)
            .stopAndAdd(common.doMoveIntake(RobotCommon.IntakeOptions.STOP))
            .stopAndAdd(common.doMoveArm(ARM_SPECIMEN))
            .strafeTo(new Vector2d(RETREAT_X, RETREAT_Y))
            .afterTime(0, common.doMoveSlides(SLIDE_SPECIMEN))
            .setTangent(Math.toRadians(180)).splineToSplineHeading(new Pose2d(CHAMBER2_X, CHAMBER2_Y, Math.toRadians(90)), Math.toRadians(90))
            .stopAndAdd(new InstantAction(() -> common.moveArm(ARM_CLIP)))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(BACK2_X, BACK2_Y))
            .afterTime(0, new SequentialAction(
                common.doMoveSlides(RobotCommon.SLIDES_RETRACTED),
                common.doMoveArm(RobotCommon.ARM_HORIZONTAL)))
            .afterTime(0, common.doMoveIntake(RobotCommon.IntakeOptions.IN))
            .setTangent(0) .splineToSplineHeading(new Pose2d(PICKUP2_X, PICKUP2_Y+3, Math.toRadians(-89.99)), Math.toRadians(-89.99))
            .waitSeconds(T_HUMAN)
            .strafeTo(new Vector2d(PICKUP2_X, PICKUP2_Y))
            .waitSeconds(T_INTAKE)
            .stopAndAdd(common.doMoveIntake(RobotCommon.IntakeOptions.STOP))
            .stopAndAdd(common.doMoveArm(ARM_SPECIMEN))
            .afterTime(0.5, common.doMoveSlides(SLIDE_SPECIMEN))
            .setTangent(Math.toRadians(180)).splineToSplineHeading(new Pose2d(CHAMBER3_X, CHAMBER3_Y, Math.toRadians(90)), Math.toRadians(90))
            .stopAndAdd(new InstantAction(() -> common.moveArm(ARM_CLIP)))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(BACK3_X, BACK3_Y))
            .endTrajectory();

        Action trajectoryAction = trajectory.build();
        preview(trajectoryAction);

        waitForStart();
        if (opModeIsActive()) {
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

            running = action.run(packet);

            common.runAuton();
            common.sendTelemetryAuton(packet);
            dash.sendTelemetryPacket(packet);
        }
    }

}

// move the arm, extend slide, drop with intake, retract slide, move arm.