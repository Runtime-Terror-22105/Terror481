package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Pose2d {
    public double x;
    public double y;
    public double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(@NonNull SparkFunOTOS.Pose2D pose) {
        this(pose.x, pose.y, pose.h);
    }

    public SparkFunOTOS.Pose2D toOtosPose() {
        return new SparkFunOTOS.Pose2D(this.x, this.y, this.heading);
    }
}
