package org.firstinspires.ftc.teamcode.math.geometry;

import org.firstinspires.ftc.teamcode.math.Coordinate;

public interface Shape {
    /**
     * Returns whether a point is on a shape.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    abstract boolean isValidSolution(Coordinate point);
}
