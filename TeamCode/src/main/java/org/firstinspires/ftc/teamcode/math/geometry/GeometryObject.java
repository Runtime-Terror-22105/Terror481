package org.firstinspires.ftc.teamcode.math.geometry;

import static org.firstinspires.ftc.teamcode.math.Algebra.sign;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.jetbrains.annotations.Contract;

public interface GeometryObject {
    /**
     * Returns whether a point is on a shape.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    abstract boolean isValidSolution(Coordinate point);
}
