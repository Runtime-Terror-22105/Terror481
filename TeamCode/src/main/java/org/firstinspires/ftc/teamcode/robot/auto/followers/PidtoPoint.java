package org.firstinspires.ftc.teamcode.robot.auto.followers;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.robot.init.Robot;

public class PidtoPoint {
    public static PidController.PidCoefficients x_coeff;
    public static PidController.PidCoefficients y_coeff;
    public static PidController.PidCoefficients heading_coeff;

    public PidController x;
    public PidController y;
    public PidController heading;


    public PidtoPoint(){
        x= new PidController(x_coeff);
        y= new PidController(y_coeff);
        heading= new PidController(heading_coeff);
    }

    public void setGoalPoint(Pose2d goal){
        x.setTargetPosition(goal.x);
        y.setTargetPosition(goal.y);
        heading.setTargetPosition(goal.heading);
    }

    public void updatePowers(Robot terrorbot, Pose2d Current){
        double x_power=x.calculatePower(Current.x);
        double y_power=y.calculatePower(Current.y);
        double heading_power=heading.calculatePower(Current.heading);
        Coordinate velocity= new Coordinate(x_power,y_power);
        terrorbot.drivetrain.move(velocity,heading_power, 1);
    }
}
