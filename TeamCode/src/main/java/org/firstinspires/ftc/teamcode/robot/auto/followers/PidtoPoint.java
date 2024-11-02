package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;

public class PidToPoint {
    public static PidController.PidCoefficients xCoeff;
    public static PidController.PidCoefficients yCoeff;
    public static PidController.PidCoefficients hCoeff;

    public PidController xController;
    public PidController yController;
    public PidController hController;

    public PidToPoint(@NonNull Pose2d goalPoint, @NonNull Pose2d tolerances) {
        xController = new PidController(xCoeff);
        yController = new PidController(yCoeff);
        hController = new PidController(hCoeff);
        this.setGoal(goalPoint, tolerances);
    }

    @NonNull
    private Pose2d calculatePower(@NonNull Pose2d currentPos) {
        double xPower = xController.calculatePower(currentPos.x);
        double yPower = yController.calculatePower(currentPos.y);
        double hPower = hController.calculatePower(currentPos.heading);
        return new Pose2d(xPower, yPower, hPower);
    }

    private boolean atTargetPosition(@NonNull Pose2d currentPos) {
        return xController.atTargetPosition(currentPos.x)
                && yController.atTargetPosition(currentPos.y)
                && hController.atTargetPosition(currentPos.heading);
    }

    public boolean driveToDestination(@NonNull Drivetrain drivetrain, @NonNull Pose2d currentPos) {
        drivetrain.move(calculatePower(currentPos));
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
