package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.auto.pathgen.QSplinePath;

import java.util.function.Supplier;

abstract class PathFollower {
    QSplinePath path;
    Drivetrain drivetrain;

    public void setDrivetrain(@NonNull Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * Sets the path to follow
     *
     * @param path The path to follow.
     * @param points How many points to generate along the path.
     */
    public void setPath(@NonNull QSplinePath path, int points) {
        this.path = path;
    }

    /**
     * Follow a path that was set by {@link #setPath(Coordinate[])} earlier.
     *
     * @param getCurrentPos A method which retrieves the current position using odometry.
     */
    abstract void follow(@NonNull Supplier<Pose2d> getCurrentPos);
}