package org.firstinspires.ftc.teamcode.math;

public class Pose2d extends Coordinate{

    public double heading;

    public Pose2d(double x, double y, double heading) {// heading is in radians
        super(x, y);
        this.heading=heading;

    }
}
