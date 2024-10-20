package org.firstinspires.ftc.teamcode.robot.drive;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Coordinate;

public interface Drivetrain {
    /**
     * Drive at a certain speed.
     * @param velocity The x and y.
     * @param rotation The rotation
     * @param speed    The robot's speed, a value between 0 and 1 (this is a multiplier, will which
     *                 basically sets the max speed).
     */
    default void move(@NonNull Coordinate velocity, double rotation, double speed) {
        velocity.mult(speed);
        rotation *= speed;
        this.move(velocity, rotation);
    };

    /**
     * Drive at max speed (1).
     * @param velocity The x and y.
     * @param rotation The rotation.
     */
    void move(@NonNull Coordinate velocity, double rotation);
}
