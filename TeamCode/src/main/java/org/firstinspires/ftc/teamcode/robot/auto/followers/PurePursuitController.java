package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.geometry.Circle;
import static org.firstinspires.ftc.teamcode.math.Coordinate.distToPoint;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.geometry.Ellipse;
import org.firstinspires.ftc.teamcode.math.geometry.LineSegment;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.auto.pathgen.QSplines;

import java.util.function.Supplier;

/**
 * A class which is used to follow paths using the Pure Pursuit algorithm.
 */
public class PurePursuitController extends PathFollower {
    public double goalThreshold;
    public double lookAheadDist;
    public Coordinate[] pathArr;

    public Obstacle[] field = new Obstacle[1000];
    public int obstacle_count=0;

    /**
     * Instantiates a pure pursuit controller.
     * @param lookAheadDist The look ahead distance.
     * @param goalThreshold The threshold before reaching.
     */
    public PurePursuitController(double lookAheadDist, double goalThreshold) {
        this.lookAheadDist = lookAheadDist;
        this.goalThreshold = goalThreshold;
    }

    public void add_Field(Obstacle obstacle){
        field[obstacle_count]=obstacle;
        obstacle_count+=1;
    }


    @Override
    public void setPath(@NonNull QSplines path, int points) {
        super.setPath(path, points);
        this.pathArr = this.path.generateArr(points);
    }

    /**
     * Follow a path that was set by {@link #setPath(Coordinate[])} earlier and drive using the
     * drivetrain set by {@link #setDrivetrain(Drivetrain)}.
     * @param getCurrentPos A method which retrieves the current position using odometry.
     */
    public void follow(@NonNull Supplier<Pose2d> getCurrentPos) {
        Coordinate lastGoalPoint = this.pathArr[0];
        int lastPathIndex = 0;

        Pose2d currentPos = getCurrentPos.get();

        while (Math.abs(distToPoint(currentPos.getVector(), this.pathArr[this.pathArr.length - 1])) > this.goalThreshold) {
            Coordinate goalPoint = null;

            for (int i = lastPathIndex; i < this.pathArr.length - 1; i++) {
                LineSegment pathSegment = new LineSegment(this.pathArr[i], this.pathArr[i+1]);
//                goalPoint = findCircleGoalPoint(pathSegment, currentPos.getVector(), this.lookAheadDist);
                goalPoint = findAdaptiveEllipseIntersection(0.5, 20, currentPos, pathSegment);

                if (goalPoint != null) {
                    lastPathIndex = i;
                    break;
                }
            }

            // If no new goal point is found, default to the last goal point
            goalPoint = (goalPoint != null) ? goalPoint : lastGoalPoint;
            lastGoalPoint = goalPoint;

//            double linearErr = distToPoint(goalPoint, currentPos.getVector());
            this.driveToPoint(currentPos, goalPoint);
            currentPos = getCurrentPos.get();
        }

        this.driveToPoint(currentPos, this.pathArr[this.pathArr.length-1]);
    }

    /**
     * Find the goal point using non-adaptive circles.
     * @param path A line segment representing the path.
     * @param robotPosition The robot position (center of the circle).
     * @param lookaheadDistance The lookahead distance (radius of the circle).
     * @return Null if no intersection is found, else the coordinate of intersection.
     */
    private Coordinate findCircleGoalPoint(LineSegment path, Coordinate robotPosition, double lookaheadDistance) {
        Circle circle = new Circle(robotPosition, lookaheadDistance);
        return circle.findNearestIntersection(path);
    }


    private void driveToPoint(@NonNull Pose2d currentPos, @NonNull Coordinate goalPoint) {
        double goalAngle = Math.atan2(goalPoint.y, goalPoint.x);
        if (goalAngle < 0) { goalAngle += 2*Math.PI; }
        double angularErr = Angle.angleWrap(goalAngle - currentPos.heading);
        drivetrain.move(
                new Coordinate(
                        goalPoint.x - currentPos.x,
                        goalPoint.y - currentPos.y
                ),
                angularErr
        );
    }

    private boolean CrashDetect(Pose2d currentPos, Coordinate goalPoint ){
        // first check if out of bounds
        for(int i = 0; i < obstacle_count; i++){ // cycling for crash with all of the obstacles
            if (field[i].crash_detect(currentPos.x,currentPos.y,goalPoint.x,goalPoint.y, currentPos.heading)) {
                return true;
            }
        }
        return false;

        // add obstacle object and use the line intersection test on all four sides of the rectangle objects bounds

    }

    public Coordinate findAdaptiveEllipseIntersection(
            double minLookAheadDist,
            double maxLookAheadDist,
            Pose2d currentPos,
            LineSegment path
    ){
        // binary search for the lookahead distance and if the lookahead doesnt work(use crash_detect) please decrease the lookahead
        double step_size = 100;
        double interval = (maxLookAheadDist-minLookAheadDist)/step_size;
        double mid = 0;
        Coordinate goalPoint = new Coordinate(0,0);
        while (minLookAheadDist <= maxLookAheadDist) {
            mid = (minLookAheadDist+maxLookAheadDist)/2;

            Ellipse robotEllipse = new Ellipse(mid, currentPos.getVector());
            robotEllipse.setRotation(currentPos.heading);
            goalPoint = robotEllipse.findNearestIntersection(path);
            if (CrashDetect(currentPos, goalPoint)) {
                maxLookAheadDist = mid-interval;
            }
            else {
                minLookAheadDist = mid+interval;
            }
        }
        return goalPoint;
    }

    public static Coordinate chooseCloserSolution(Coordinate sol1, Coordinate sol2, Coordinate target) {
        if (sol1 != null && sol2 != null) {
            return Coordinate.distToPoint(sol1, target) < Coordinate.distToPoint(sol2, target) ? sol1 : sol2;
        } else if (sol1 != null) {
            return sol1;
        } else {
            return sol2;
        }
    }


}
