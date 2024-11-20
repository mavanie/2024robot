package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class RoadRunnerSample extends LinearOpMode {
    RobotCommon common;
    private ElapsedTime opModeTime = new ElapsedTime();
    private IMU imu;
    private double yawOffset;


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectory = drive.actionBuilder(initialPose)
            .turnTo(Math.toRadians(180));

        Action trajectoryAction = trajectory.build();
        preview(trajectoryAction);

        waitForStart();
        opModeTime.reset();
        while (opModeIsActive()) {
            initialPose = new Pose2d(0, 0, 0);
            drive = new MecanumDrive(hardwareMap, initialPose);

            trajectory = drive.actionBuilder(initialPose)
                .turnTo(Math.toRadians(180))
                .waitSeconds(4);

            trajectoryAction = trajectory.build();
            runBlocking(trajectoryAction);
        }
    }

    private void initialize(){
        common = new RobotCommon(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        common.initialize();
        imu = hardwareMap.get(IMU.class, "imuExpansion");
        yawOffset = imu.getRobotYawPitchRollAngles().getYaw();
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
        packet.put("yaw", 0);

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
            packet.put("yaw", imu.getRobotYawPitchRollAngles().getYaw() - yawOffset);

            running = action.run(packet);

            common.runAuton();
            common.sendTelemetryAuton(packet);
            dash.sendTelemetryPacket(packet);
        }
    }
}
