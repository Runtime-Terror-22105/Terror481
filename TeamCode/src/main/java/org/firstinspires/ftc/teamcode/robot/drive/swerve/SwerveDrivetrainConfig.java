package org.firstinspires.ftc.teamcode.robot.drive.swerve;

/**
 * Configuration for the swerve drivetrain.
 */
public class SwerveDrivetrainConfig {
    public final double wheelBase; // The wheel base, usually called "L" in equations
    public final double trackWidth; // The track width, usually called "W" in equations
    public final double R; // idk what else to call this

    /**
     * Initializes a SwerveDrivetrainConfig class.
     * @param wheelBase            The robot's wheelbase (L), must be the same units as the track width
     * @param trackWidth           The robot's trackwidth (W), must be the same units as the wheelbase
     */
    public SwerveDrivetrainConfig(double wheelBase, double trackWidth) {
        this.wheelBase = wheelBase;
        this.trackWidth = trackWidth;
        this.R = Math.sqrt(Math.pow(this.wheelBase,2) + Math.pow(this.trackWidth,2));
    }

}
