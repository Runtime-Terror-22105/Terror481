package org.firstinspires.ftc.teamcode.math;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;

public class Pose2d {
    public double x;
    public double y;
    public double heading;



    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Add two poses.
     * @param v1 The first pose
     * @param v2 The second pose
     * @return A pose, with the elements of both added to each other
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Pose2d plus(@NonNull Pose2d v1, @NonNull Pose2d v2) {
        return new Pose2d(v1.x + v2.x, v1.y + v2.y, v1.heading + v2.heading);
    }

    /**
     * Add two poses.
     * @param other The pose to add
     * @return A new pose, with the elements of both added to each other
     */
    public Pose2d plus(@NonNull Pose2d other) {
        this.x += other.x;
        this.y += other.y;
        this.heading += other.heading;
        return this;
    }

    /**
     * Add two poses.
     * @param v1 The first pose
     * @param v2 The second pose
     * @return A pose, with the elements of both added to each other
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Pose2d minus(@NonNull Pose2d v1, @NonNull Pose2d v2) {
        return new Pose2d(v1.x - v2.x, v1.y - v2.y, v1.heading - v2.heading);
    }

    /**
     * Add two poses.
     * @param other The pose to add
     * @return A new pose, with the elements of both added to each other
     */
    public Pose2d minus(@NonNull Pose2d other) {
        this.x -= other.x;
        this.y -= other.y;
        this.heading -= other.heading;
        return this;
    }

    /**
     * Multiply a pose by a scalar.
     * @param v1 The pose to scale
     * @param scalar The scalar to multiply by
     * @return The scaled pose
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Pose2d mult(@NonNull Pose2d v1, double scalar) {
        return new Pose2d(v1.x * scalar, v1.y * scalar, v1.heading * scalar);
    }

    /**
     * Multiply a pose by a scalar.
     * @param scalar The scalar to multiply by
     * @param v1 The pose to scale
     * @return The scaled pose
     */
    @NonNull
    @Contract(value = "_, _ -> new", pure = true)
    public static Pose2d mult(double scalar, @NonNull Pose2d v1) {
        return new Pose2d(v1.x * scalar, v1.y * scalar, v1.heading * scalar);
    }

    /**
     * Multiply the pose by a scalar.
     * @param scalar The scalar to multiply by
     * @return The scaled pose
     */
    public Pose2d mult(@NonNull double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.heading *= scalar;
        return this;
    }

    /**
     * Gets the x and y in a Vector2.
     * @return A Vector2.
     */
    public Coordinate getVector() {
        return new Coordinate(this.x, this.y);
    }
}
