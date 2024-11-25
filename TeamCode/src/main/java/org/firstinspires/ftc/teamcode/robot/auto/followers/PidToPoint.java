package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

@Config
public class PidToPoint {
    public static PidController.PidCoefficients xCoeff = new PidController.PidCoefficients(0.43, 0, 0.04);
    public static PidController.PidCoefficients yCoeff = new PidController.PidCoefficients(0.19, 0, 0.018);
    public static PidController.PidCoefficients hCoeff = new PidController.PidCoefficients(2, 0, 0.11);

    public PidController xController;
    public PidController yController;
    public PidController hController;

    public double xTemp;
    public double yTemp;

    /**
     * The amount of milliseconds that the robot needs to be at its destination for it to count
     * as "reaching" its destination.
     */
    private double reachedTime;

    private double lastReachedTime = 0;

    public PidToPoint() {
        this(
                new Pose2d(0,0,0),
                new Pose2d(0,0,0),
                100
        );
    }

    public PidToPoint(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances, double reachedTime) {
        xController = new PidController(xCoeff);
        yController = new PidController(yCoeff);
        hController = new PidController(hCoeff);
        this.setGoal(goalPoint, tolerances, reachedTime);
    }

    @NonNull
    public Pose2d calculatePower(@NonNull Pose2d currentPos) {
        this.xTemp = xController.calculatePower(currentPos.x);
        this.yTemp = yController.calculatePower(currentPos.y);
        double h = hController.calculatePower(currentPos.heading, true);

        double angle = -currentPos.heading;
        double x = xTemp * Math.cos(angle) - yTemp * Math.sin(angle);
        double y = xTemp * Math.sin(angle) + yTemp * Math.cos(angle);
        return new Pose2d(x, y, h);
    }

    private boolean atTargetPosition(@NonNull Pose2d currentPos) {
        return xController.atTargetPosition(currentPos.x)
                && yController.atTargetPosition(currentPos.y)
                && hController.atTargetPosition(currentPos.heading);
    }

    public boolean driveToDestination(@NonNull Drivetrain drivetrain, @NonNull Pose2d powers, @NonNull Pose2d currentPos) {
        drivetrain.move(powers);
        if (this.atTargetPosition(currentPos)) {
            if (this.lastReachedTime == 0) {
                // we just reached our target
                this.lastReachedTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - this.lastReachedTime >= this.reachedTime) {
                // we've spent the required amount of time at the destination
                return true;
            }
        } else {
            // we haven't reached the destination yet
            this.lastReachedTime = 0;
        }

        return false;
    }

    public void setGoal(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances, double reachedTime) {
        xController.setTargetPosition(goalPoint.x);
        yController.setTargetPosition(goalPoint.y);
        hController.setTargetPosition(goalPoint.heading);

        xController.setTolerance(tolerances.x);
        yController.setTolerance(tolerances.y);
        hController.setTolerance(tolerances.heading);

        this.reachedTime = reachedTime;
    }

    public Pose2d getError() {
        return new Pose2d(
                xController.getLastError(),
                yController.getLastError(),
                hController.getLastError()
        );
    }
}
