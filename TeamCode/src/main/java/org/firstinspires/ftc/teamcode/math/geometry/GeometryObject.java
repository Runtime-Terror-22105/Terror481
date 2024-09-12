package org.firstinspires.ftc.teamcode.math.geometry;

import static org.firstinspires.ftc.teamcode.math.Algebra.sign;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.jetbrains.annotations.Contract;

public abstract class GeometryObject {
    /**
     * Returns whether a point is on a shape.
     * @param point The point.
     * @return Whether or not the point sits on the shape.
     */
    abstract boolean isValidSolution(Coordinate point);

    /**
     * Get an intersection between a line segment and circle. If there are two, the point closer
     * to the second point will be chosen. If there are none, null will be returned.
     * @param point1 First point on line
     * @param point2 Second point on line
     * @param center Center of circle
     * @param radius Radius of circle
     * @return A point, can be null.
     */
    @Nullable
    @Contract("_, _, _, _ -> new")
    public static Coordinate circleLineIntersection(@NonNull Coordinate point1, @NonNull Coordinate point2, Coordinate center, double radius) {
        // Math comes from https://mathworld.wolfram.com/Circle-LineIntersection.html, except we
        // shift the center of the circle to the origin since our circle won't be centered at the
        // origin. We also add an extra check that the circle intersects with the line segmnet, not
        // the entire line.
        Coordinate p1shifted = Coordinate.minus(point1, center); // shift to origin
        Coordinate p2shifted = Coordinate.minus(point2, center); // shift to origin

        double minX = Math.min(point1.x, point2.x);
        double maxX = Math.max(point1.x, point2.x);
        double minY = Math.min(point1.y, point2.y);
        double maxY = Math.max(point1.y, point2.y);

        double dx = point2.x - point1.x;
        double dy = point2.y - point1.y;
        double dr = Math.sqrt(dx * dx + dy * dy);
        double det = p1shifted.x * p2shifted.y - p2shifted.x * p1shifted.y;
        double discriminant = Math.pow(radius, 2) * Math.pow(dr, 2) - Math.pow(det, 2);

        if (discriminant < 0) {
            return null;
        }

        double temp = sign(dy) * dx * Math.sqrt(discriminant);
        double x1 = (det * dy + temp) / Math.pow(dr, 2);
        double x2 = (det * dy - temp) / Math.pow(dr, 2);

        temp = Math.abs(dy) * Math.sqrt(discriminant);
        double y1 = (-det * dx + temp) / Math.pow(dr, 2);
        double y2 = (-det * dx - temp) / Math.pow(dr, 2);

        Coordinate sol1 = new Coordinate(x1, y1).plus(center);
        Coordinate sol2 = new Coordinate(x2, y2).plus(center);

        // Set the solution to null if it is outside of the line segment
        if (!Coordinate.isWithinBounds(sol1, minX, maxX, minY, maxY)) {
            sol1 = null;
        }
        if (!Coordinate.isWithinBounds(sol2, minX, maxX, minY, maxY)) {
            sol2 = null;
        }

        // Return the valid solution, or if there are two, the one closer to point 2
        return point2.closestPoint(sol1, sol2);
    }
}
