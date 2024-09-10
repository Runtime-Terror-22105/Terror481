package org.firstinspires.ftc.teamcode.math;

import static org.firstinspires.ftc.teamcode.util.math.Algebra.sign;

public class Angle {
    /**
     * Do the angle optimization for swerve.
     * @return The optimized angle.
     */
    public static double optimize(double angle, double goalAngle) {
        if (Angle.canBeOptimized(angle, goalAngle)) {
            return Angle.angleWrap(angle - Math.PI);
        }
        return angle;
    }

    public static boolean canBeOptimized(double angle, double goalAngle) {
        double error = Angle.angleWrap(goalAngle - angle);
        return Math.abs(error) > Math.PI / 2;
    }

    /**
     * Change an angle to be between 0 and 360 degrees.
     * @param angle An angle, in radians.
     * @return The normalized angle.
     */
    public static double normalize(double angle) {
        while (angle > 2*Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Gets the difference between two angles.
     * @param angle1 The first angle
     * @param angle2 The second angle
     * @return The difference between the two angles, in radians
     */
    public static double getdelta(double angle1, double angle2){ // function for difference between 2 angles
        double[] changes = {-2.0*Math.PI, 0.0, 2.0*Math.PI}; // add/subtract 2pi radians
        double min_delta = Math.abs(angle2 - angle1);
        for (int i=0; i < 3; i++) {
            min_delta = Math.min(min_delta, Math.abs((angle2 + changes[i]) - angle1));
        }
        return min_delta;
    }

    public static double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}
