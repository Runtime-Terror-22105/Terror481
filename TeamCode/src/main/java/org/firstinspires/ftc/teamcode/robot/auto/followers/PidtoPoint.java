package org.firstinspires.ftc.teamcode.robot.auto.followers;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class PidToPoint {
    public static PidController.PidCoefficients xCoeff;
    public static PidController.PidCoefficients yCoeff;
    public static PidController.PidCoefficients hCoeff;

    public PidController xController;
    public PidController yController;
    public PidController hController;


    public PidToPoint() {
        this.xController = new PidController(xCoeff);
        this.yController = new PidController(yCoeff);
        this.hController = new PidController(hCoeff);
    }

    public void setGoalPoint(@NonNull Pose2d goal) {
        xController.setTargetPosition(goal.x);
        yController.setTargetPosition(goal.y);
        hController.setTargetPosition(goal.heading);
    }

    public Pose2d calculatePower(@NonNull Robot terrorbot, @NonNull Pose2d Current){
        double xPower = xController.calculatePower(Current.x);
        double yPower = yController.calculatePower(Current.y);
        double hPower = hController.calculatePower(Current.heading);
        return new Pose2d(xPower, yPower, hPower);
    }
}
