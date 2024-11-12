package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            IMU imu = hardwareMap.get(IMU.class, "imuExpansion");

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.guide) {
                    imu.resetYaw();
                }
                double factor = (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) ? 1 : 4;
                double x = -square(gamepad1.left_stick_y)/factor;
                double y = -square(gamepad1.left_stick_x)/factor;
                double rot = -square(gamepad1.right_trigger - gamepad1.left_trigger);
                double yaw = -imu.getRobotYawPitchRollAngles().getYaw();
                double vx = x * Math.cos(Math.toRadians(yaw))- y * Math.sin(Math.toRadians(yaw));
                double vy = x * Math.sin(Math.toRadians(yaw)) + y * Math.cos(Math.toRadians(yaw));

//                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(vx, vy), rot));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                // match our field orientation
                Canvas c = packet.fieldOverlay();
//                c.setRotation(Math.toRadians(-90)).setTranslation(0, 72);
                c.setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//                c.strokeLine(0, 0, 24, 48);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                0.0
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
    public static double square(double amount) {
        return amount * Math.abs(amount);
    }
}
