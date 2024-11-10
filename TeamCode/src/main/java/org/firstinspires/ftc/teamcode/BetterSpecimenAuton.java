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
    public static Vector2d CHAMBER = new Vector2d(29, -9);
    public static Vector2d BACK = new Vector2d(20, -9);
    public static double ARM_SPECIMEN = 0.7;
    public static int SLIDE_SPECIMEN = 1000;
    public static double ARM_CLIP = 1.2;
    public static double T_CLIP = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = START;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .stopAndAdd(common.doMoveArm(ARM_SPECIMEN))
            .strafeTo(CHAMBER)
            .afterTime(0, common.doMoveSlides(SLIDE_SPECIMEN))
            .stopAndAdd(() -> common.moveArm(ARM_CLIP)) // don't wait for the arm to reach its target
            .waitSeconds(T_CLIP)
            .strafeTo(BACK)
            .endTrajectory();

        waitForStart();
        if (opModeIsActive()) {
            runBlocking(new SequentialAction(
                trajectory.build()
            ));
        }
    }

    private void initialize(){
        common = new RobotCommon(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common.initialize();
        telemetry.addData("x", 0);
        telemetry.addData("y", 0);
        telemetry.addData("xError", 0);
        telemetry.addData("yError", 0);
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
//            sendTelemetry();
        }
    }

    private void sendTelemetry(){
        common.sendTelemetry(telemetry);
        telemetry.addData("x", 0);

        telemetry.update();
    }
}

// move the arm, extend slide, drop with intake, retract slide, move arm.