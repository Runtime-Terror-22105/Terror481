package org.firstinspires.ftc.teamcode.robot.drive.swerve;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.controllers.SwervePidfController;


/**
 * A class which stores the configuration for a swerve module and can be easily edited through
 * dashboard.
 */
public class SwerveModuleConfig {
    public final SwervePidfController.SwervePidfCoefficients anglePidfCoefficientsCCW;
    public final SwervePidfController.SwervePidfCoefficients anglePidfCoefficientsCW;
    public final Coordinate moduleOffset; // the offset from the center
    public final boolean servoEncoderReversed;
    public final double servoEncoderOffset;

    /**
     * Initializes a SwerveModuleConfig class.
     * @param anglePidfCoefficientsCCW The coefficients for the pid of the module's rotation when rotating ccw
     * @param anglePidfCoefficientsCW The coefficients for the pid of the module's rotation when rotating cw
     * @param servoEncoderReversed Whether or not to reverse the encoder for the servo which does
     *                             the turning
     * @param servoEncoderOffset   How much to offset the encoder for the turning servo to make it
     *                             be 0.
     */
    public SwerveModuleConfig(SwervePidfController.SwervePidfCoefficients anglePidfCoefficientsCCW,
                              SwervePidfController.SwervePidfCoefficients anglePidfCoefficientsCW,
                              boolean servoEncoderReversed, Coordinate moduleOffset,
                              double servoEncoderOffset) {
        this.anglePidfCoefficientsCCW = anglePidfCoefficientsCCW;
        this.anglePidfCoefficientsCW = anglePidfCoefficientsCW;
        this.servoEncoderReversed = servoEncoderReversed;
        this.moduleOffset = moduleOffset;
        this.servoEncoderOffset = servoEncoderOffset;
    }
}
