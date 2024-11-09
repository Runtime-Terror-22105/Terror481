package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

@Config
public class PidToPoint {
    public static PidController.PidCoefficients xCoeff = new PidController.PidCoefficients(0, 0, 0);
    public static PidController.PidCoefficients yCoeff = new PidController.PidCoefficients(0, 0, 0);
    public static PidController.PidCoefficients hCoeff = new PidController.PidCoefficients(2, 0, 0.11);

    public PidController xController;
    public PidController yController;
    public PidController hController;

    public PidToPoint() {
        this(
                new Pose2d(0,0,0),
                new Pose2d(0,0,0)
        );
    }

    public PidToPoint(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances) {
        xController = new PidController(xCoeff);
        yController = new PidController(yCoeff);
        hController = new PidController(hCoeff);
        this.setGoal(goalPoint, tolerances);
    }

    @NonNull
    public Pose2d calculatePower(@NonNull Pose2d currentPos) {
        double xTemp = xController.calculatePower(currentPos.x);
        double yTemp = yController.calculatePower(currentPos.y);
        double angle = hController.calculatePower(currentPos.heading, true);

        double x = xTemp * Math.cos(angle) - yTemp * Math.sin(angle);
        double y = xTemp * Math.sin(angle) + yTemp * Math.cos(angle);
        return new Pose2d(x, y, angle);
    }

    private boolean atTargetPosition(@NonNull Pose2d currentPos) {
        return xController.atTargetPosition(currentPos.x)
                && yController.atTargetPosition(currentPos.y)
                && hController.atTargetPosition(currentPos.heading);
    }

    public boolean driveToDestination(@NonNull Drivetrain drivetrain, @NonNull Pose2d powers, @NonNull Pose2d currentPos) {
        drivetrain.move(powers);
        return this.atTargetPosition(currentPos);
    }

    public void setGoal(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances) {
        xController.setTargetPosition(goalPoint.x);
        yController.setTargetPosition(goalPoint.y);
        hController.setTargetPosition(goalPoint.heading);

        xController.setTolerance(tolerances.x);
        yController.setTolerance(tolerances.y);
        hController.setTolerance(tolerances.heading);
    }

    public Pose2d getError() {
        return new Pose2d(
                xController.getLastError(),
                yController.getLastError(),
                hController.getLastError()
        );
    }
}
