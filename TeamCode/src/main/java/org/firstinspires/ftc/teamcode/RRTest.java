package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp
@Config
public class RRTest extends LinearOpMode {
    private RobotCommon common;
    public static double X1 = 48;
    public static double Y1 = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();
        if (opModeIsActive()) {
            common.doMoveArm(1.5);
            while (opModeIsActive()) {
                TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(X1, Y1))
                        .strafeTo(new Vector2d(0, 0))
                        .endTrajectory();
                runBlocking(trajectory.build());
            }
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
        packet.put("heading (deg)", 0);
        packet.put("headingError (deg)", 0);
        packet.put("x", 0);
        packet.put("xError", 0);
        packet.put("y", 0);
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

            running = action.run(packet);

            common.runAuton();
            common.sendTelemetryAuton(packet);
            dash.sendTelemetryPacket(packet);
        }
    }

}
