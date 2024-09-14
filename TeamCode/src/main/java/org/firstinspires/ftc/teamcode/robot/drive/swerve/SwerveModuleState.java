package org.firstinspires.ftc.teamcode.robot.drive.swerve;

public class SwerveModuleState {
    public double velocity;
    public double angle;

    /**
     * Store the velocity to drive at and the angle for the wheel.
     * @param velocity The power for the speed servo
     * @param angle The angle, in radians
     */
    public SwerveModuleState(double velocity, double angle) {

        
        this.velocity = velocity;
        this.angle = angle;
    }
    public SwerveModuleState() {
        this(0, 0);
    }
}
