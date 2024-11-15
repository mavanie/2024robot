package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Config
@Autonomous(name = "BetterSpecimenAuton", preselectTeleOp = "DriverControl")
public class BetterSpecimenAuton extends LinearOpMode {
    private RobotCommon common;
    private ElapsedTime opModeTime = new ElapsedTime();
    public static double START_X = 8.75;
    public static double START_Y = -62.75;
    public static double START_R = 90;
    public static double CHAMBER_X = 8.75;
    public static double CHAMBER_Y = -37.5;
    public static double BACK_X = 8.75;
    public static double BACK_Y = -48;
    public static double PICKUP1_X = 47;
    public static double PICKUP1_Y = -42.5;
    public static double PICKUP_PUSH = 7;
    public static double CHAMBER2_X = 0;
    public static double CHAMBER2_Y = -38;
    public static double BACK2_X = 4.4;
    public static double BACK2_Y = -48;
    public static double SIDE1_X = 36;
    public static double SIDE1_Y = -36;
    public static double FORWARD1_X = 36;
    public static double FORWARD1_Y = -12;
    public static double SIDE2_X = 46;
    public static double SIDE2_Y = -12;
    public static double BACK3_X = 46;
    public static double BACK3_Y = -60;
    public static double FORWARD2_X = 46;
    public static double FORWARD2_Y = -12;
    public static double SIDE3_X = 56;
    public static double SIDE3_Y = -12;
    public static double BACK4_X = 56;
    public static double BACK4_Y = -60;
    public static double FORWARD3_X = 56;
    public static double FORWARD3_Y = -12;
    public static double SIDE4_X = 64;
    public static double SIDE4_Y = -12;
    public static double BACK5_X = 64;
    public static double BACK5_Y = -60;
    public static double ARM_SPECIMEN = 0.7;
    public static double ARM_PICKUP = 1.45;
    public static int SLIDE_SPECIMEN = 1000;
    public static double ARM_CLIP = 1.2;
    public static double T_CLIP = 1;
    public static double T_START = 0.2;
    public static double V_MEDIUM = 15;
    public static double V_SLOW = 7;
    public static double R_SLOW = 1.8;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        VelConstraint medium = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(V_MEDIUM),
            new AngularVelConstraint(R_SLOW)
        ));
        Pose2d initialPose = new Pose2d(START_X, START_Y, Math.toRadians(START_R));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .afterTime(0, common.doMoveArm(ARM_SPECIMEN))
            .afterTime(0.2, common.doMoveSlides(SLIDE_SPECIMEN))
            .waitSeconds(T_START)
            .strafeTo(new Vector2d(CHAMBER_X, CHAMBER_Y), medium)
            .stopAndAdd(new SequentialAction(
                new InstantAction(() -> common.moveArm(ARM_CLIP)) // don't wait for the arm to reach its target
            ))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(BACK_X, BACK_Y), medium)
            .afterTime(0, new ParallelAction(
//                common.doMoveSlides(SLIDE_PICKUP),
                common.doMoveIntake(RobotCommon.IntakeOptions.IN),
                common.doMoveArm(ARM_PICKUP)
                ))
            .strafeToLinearHeading(new Vector2d(PICKUP1_X, PICKUP1_Y), Math.toRadians(-90),medium)
            .strafeTo(new Vector2d(PICKUP1_X, PICKUP1_Y - PICKUP_PUSH), new TranslationalVelConstraint(V_SLOW))
            .stopAndAdd(common.doMoveIntake(RobotCommon.IntakeOptions.STOP))
            .stopAndAdd(common.doMoveArm(ARM_SPECIMEN))
//            .afterTime(0, common.doMoveSlides(SLIDE_SPECIMEN))
            .setTangent(Math.toRadians(180)).splineToSplineHeading(new Pose2d(CHAMBER2_X, CHAMBER2_Y, Math.toRadians(90)), Math.toRadians(90))
            .stopAndAdd(new InstantAction(() -> common.moveArm(ARM_CLIP)))
            .waitSeconds(T_CLIP)
            .strafeTo(new Vector2d(BACK2_X, BACK2_Y))
            .strafeTo(new Vector2d(SIDE1_X, SIDE1_Y))
            .strafeTo(new Vector2d(FORWARD1_X, FORWARD1_Y))
            .strafeTo(new Vector2d(SIDE2_X, SIDE2_Y))
            .strafeTo(new Vector2d(BACK3_X, BACK3_Y))
            .strafeTo(new Vector2d(FORWARD2_X, FORWARD2_Y))
            .strafeTo(new Vector2d(SIDE3_X, SIDE3_Y))
            .strafeTo(new Vector2d(BACK4_X, BACK4_Y))
            .strafeTo(new Vector2d(FORWARD3_X, FORWARD3_Y))
            .strafeTo(new Vector2d(SIDE4_X, SIDE4_Y))
            .strafeTo(new Vector2d(BACK5_X, BACK5_Y))
            .endTrajectory();

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