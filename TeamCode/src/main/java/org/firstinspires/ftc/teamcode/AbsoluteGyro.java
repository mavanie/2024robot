package org.firstinspires.ftc.teamcode;

public class AbsoluteGyro {
    private double previousYaw = 0;
    private int turns = 0;

    public double calculate(double yaw) {
        if (yaw > 170 && previousYaw < -170) {
            turns++;
        } else if (yaw < -170 && previousYaw > 170) {
            turns--;
        }
        previousYaw=yaw;
        return yaw - turns * 360;
    }
}