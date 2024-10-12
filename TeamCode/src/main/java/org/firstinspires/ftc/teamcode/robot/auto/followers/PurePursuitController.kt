package org.firstinspires.ftc.teamcode.robot.auto.followers

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import org.firstinspires.ftc.teamcode.math.Angle
import org.firstinspires.ftc.teamcode.math.Coordinate
import org.firstinspires.ftc.teamcode.math.geometry.Circle
import org.firstinspires.ftc.teamcode.math.geometry.Ellipse
import org.firstinspires.ftc.teamcode.math.geometry.LineSegment
import org.firstinspires.ftc.teamcode.robot.auto.pathgen.QSplines
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain
import java.util.function.Supplier
import kotlin.math.abs

/**
 * A class which is used to follow paths using the Pure Pursuit algorithm.
 */
class PurePursuitController (var goalThreshold: Double) {
    lateinit var pathArr: Array<Coordinate>
    var field = arrayOfNulls<Obstacle>(1000)
    var obstacle_count = 0

    fun add_Field(obstacle: Obstacle?) {
        field[obstacle_count] = obstacle
        obstacle_count += 1
    }

    var path: QSplines? = null
    var drivetrain: Drivetrain? = null
    fun setDrivetrain(drivetrain: Drivetrain) {
        this.drivetrain = drivetrain
    }

    /**
     * Sets the path to follow
     *
     * @param path The path to follow.
     * @param points How many points to generate along the path.
     */
    fun setPath(path: QSplines, points: Int) {
        this.pathArr = path.generateArr(points);
    }

    /**
     * Follow a path that was set by [.setPath] earlier and drive using the
     * drivetrain set by [.setDrivetrain].
     * @param getCurrentPos A method which retrieves the current position using odometry.
     */
    fun follow(getCurrentPos: Supplier<SparkFunOTOS.Pose2D>) {
        var lastGoalPoint: Intersection = Intersection(pathArr[0].toPose(), 0)
        var lastPathIndex = 0
        var currentPos = getCurrentPos.get()

        while (abs(Coordinate.distToPoint(Coordinate.fromPose(currentPos), pathArr[pathArr.size - 1])) > goalThreshold) {
            val goalPoint: Intersection = findAdaptiveEllipseIntersection(
                lastPathIndex,
                0.5,
                5.0,
                currentPos
            ) ?: lastGoalPoint  // If no new point is found, default to the last goal point
            lastGoalPoint = goalPoint
            lastPathIndex = goalPoint.pathSegmentIndex

//            double linearErr = distToPoint(goalPoint, currentPos.getVector());
            driveToPoint(currentPos, goalPoint.point)
            currentPos = getCurrentPos.get()
        }
        driveToPoint(currentPos, pathArr[pathArr.size - 1].toPose())
    }

    /**
     * This closed-loop follower uses the standard pure pursuit controller with a fixed circular
     * lookahead. <br>
     * Do not use this, it is merely for demonstration.
     *
     * @param getCurrentPos A method which retrieves the current position of the robot.
     * @param lookAheadDistance The lookahead distance (radius of the circle).
     */
    fun normalFollow(getCurrentPos: Supplier<SparkFunOTOS.Pose2D>, lookAheadDistance: Double) {
        var lastGoalPoint: Coordinate? = pathArr[0]
        var lastPathIndex = 0
        var currentPos = getCurrentPos.get()
        while (abs(Coordinate.distToPoint(Coordinate.fromPose(currentPos), pathArr[pathArr.size - 1])) > goalThreshold) {
            var goalPoint: Coordinate? = null
            for (i in lastPathIndex until pathArr.size - 1) {
                val pathSegment = LineSegment(pathArr[i], pathArr[i + 1])
                goalPoint = findCircleGoalPoint(
                    pathSegment,
                    Coordinate.fromPose(currentPos),
                    lookAheadDistance
                )
                if (goalPoint != null) {
                    lastPathIndex = i
                    break
                }
            }

            // If no new goal point is found, default to the last goal point
            goalPoint = goalPoint ?: lastGoalPoint
            lastGoalPoint = goalPoint

//            double linearErr = distToPoint(goalPoint, currentPos.getVector());
            driveToPoint(currentPos, goalPoint!!.toPose())
            currentPos = getCurrentPos.get()
        }
        driveToPoint(currentPos, pathArr[pathArr.size - 1].toPose())
    }

    /**
     * Find the goal point using non-adaptive circles.
     * @param path A line segment representing the path.
     * @param robotPosition The robot position (center of the circle).
     * @param lookaheadDistance The lookahead distance (radius of the circle).
     * @return Null if no intersection is found, else the coordinate of intersection.
     */
    private fun findCircleGoalPoint(
        path: LineSegment,
        robotPosition: Coordinate,
        lookaheadDistance: Double
    ): Coordinate? {
        val circle = Circle(robotPosition, lookaheadDistance)
        return circle.findNearestIntersection(path)
    }

    private fun driveToPoint(currentPos: SparkFunOTOS.Pose2D, goalPoint: SparkFunOTOS.Pose2D) {
        if (goalPoint.h < 0) {
            goalPoint.h += 2 * Math.PI
        }
        val angularErr = Angle.angleWrap(goalPoint.h - currentPos.h)
        drivetrain?.move(
            Coordinate(
                goalPoint.x - currentPos.x,
                goalPoint.y - currentPos.y
            ),
            angularErr
        )
    }

    private fun crashDetect(currentPos: SparkFunOTOS.Pose2D, goalPoint: Coordinate): Boolean {
        // first check if out of bounds
        for (i in 0 until obstacle_count) { // cycling for crash with all of the obstacles
            if (field[i]!!
                    .crash_detect(
                        currentPos.x,
                        currentPos.y,
                        goalPoint.x,
                        goalPoint.y,
                        currentPos.h
                    )
            ) {
                return true
            }
        }
        return false

        // add obstacle object and use the line intersection test on all four sides of the rectangle objects bounds
    }

    private fun findAdaptiveEllipseIntersection(
        minIndex: Int,
        minLookAhead: Double,
        maxLookAhead: Double,
        currentPos: SparkFunOTOS.Pose2D
    ): Intersection? {
        var minLookAhead = minLookAhead
        var maxLookAhead = maxLookAhead

        // binary search for the lookahead distance and if the lookahead doesnt work(use crash_detect) please decrease the lookahead
        val stepSize = 100.0
        val interval = ((maxLookAhead - minLookAhead).toDouble() / stepSize).toInt()
        var midLookAhead = 0.0

        var goalPoint: Coordinate? = null
        var foundPathIndex = -1
        while (minLookAhead <= maxLookAhead) {
            midLookAhead = (minLookAhead + maxLookAhead) / 2
            val robotEllipse = Ellipse(
                Coordinate.fromPose(currentPos),
                midLookAhead,
                midLookAhead*0.5, // TODO: determine minor axis
                currentPos.h
            )

            var intersection: Coordinate? = null
            for (i in minIndex until pathArr.size - 1) {
                val pathSegment = LineSegment(pathArr[i], pathArr[i + 1])
                intersection = robotEllipse.findNearestIntersection(pathSegment)

                if (intersection != null) {
                    foundPathIndex = i
                    break
                }
            }

            goalPoint = intersection
            if (crashDetect(currentPos, goalPoint)) {
                maxLookAhead = midLookAhead - interval
            } else {
                minLookAhead = midLookAhead + interval
            }
        }
        if (goalPoint == null) {
            return null
        }
        return Intersection(goalPoint.toPose(), foundPathIndex)
    }

    private class Intersection(val point: SparkFunOTOS.Pose2D, val pathSegmentIndex: Int)
}
