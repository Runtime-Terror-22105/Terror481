package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.math.controllers.PidController;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

/**
 * A class representing a telescoping arm.
 */
public class PinkArm {
    // The hardware
    private final TerrorMotor armPitchMotor1;
    private final TerrorMotor armPitchMotor2;
    private final TerrorAnalogEncoder armPitchEncoder;
    private final TerrorMotor armExtensionMotor1;
    private final TerrorMotor armExtensionMotor2;
    private final TerrorEncoder armExtensionEncoder;

    // PIDs
    public static PidfController.PidfCoefficients pitchPidCoefficients =
            new PidfController.PidfCoefficients(0, 0, 0, 0, 0);
    private final PidfController pitchPid = new PidfController(pitchPidCoefficients);
    public static PidController.PidCoefficients extensionPidCoefficients =
            new PidController.PidCoefficients(0, 0, 0);
    private final PidController extensionPid = new PidController(extensionPidCoefficients);

    /**
     * Creates a new pink arm
     * @param hardware The robot's hardware devices
     */
    public PinkArm(@NonNull RobotHardware hardware) {
        this.armPitchMotor1 = hardware.armPitchMotor1;
        this.armPitchMotor2 = hardware.armPitchMotor2;
        this.armPitchEncoder = hardware.armPitchEncoder;
        this.armExtensionMotor1 = hardware.armExtensionMotor1;
        this.armExtensionMotor2 = hardware.armExtensionMotor2;
        this.armExtensionEncoder = hardware.armExtensionEncoder;
    }

    /**
     * Sets the desired pitch of the pink arm
     * @param angle The desired pitch, in radians
     */
    public void setPitch(double angle) {
        pitchPid.setTargetPosition(angle);
    }

    /**
     * Sets the desired extension of the pink arm
     * @param distance The desired extension, in ticks
     */
    public void setExtension(double distance) {
        extensionPid.setTargetPosition(distance);
    }

    /**
     * Updates the motors base on the desired pitch and extension.
     * Run this function every loop iteration.
     */
    public void update() {
        double pitchPower = pitchPid.calculatePower(armPitchEncoder.getCurrentPosition(), 0);
        this.armPitchMotor1.setPower(pitchPower);
        this.armPitchMotor2.setPower(pitchPower);

        double extensionPower = extensionPid.calculatePower(armExtensionEncoder.getCurrentPosition());
        this.armExtensionMotor1.setPower(extensionPower);
        this.armExtensionMotor2.setPower(extensionPower);
    }
}
