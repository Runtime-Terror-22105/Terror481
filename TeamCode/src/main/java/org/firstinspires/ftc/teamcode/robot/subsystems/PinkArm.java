package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorAnalogEncoder;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorEncoder;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

/**
 * A class representing a telescoping arm.
 */
@Config
public class PinkArm {
    // The hardware
    private final TerrorMotor armPitchMotor1;
    private final TerrorMotor armPitchMotor2;
    private final TerrorAnalogEncoder armPitchEncoder;
    private final TerrorMotor armExtensionMotor1;
    private final TerrorMotor armExtensionMotor2;
    private final TerrorEncoder armExtensionEncoder;

    /**
     * Max extension at any point, not just horizontal, used to calculate feedforward
     */
    public static final double MAX_EXTENSION = 450;

    /**
     * Max pitch in radians
     */
    public static final double maxPitch = 1.62316;

    /**
     * Value 1 from "PitchFFTuner.java"
     */
    public static double value1 = 0.03;

    /**
     * Value 2 from "PitchFFTuner.java"
     */
    public static double value2 = 0.08;

    // PIDs
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)

    /**
     * Tune this value from "ExtensionPIDTuner.java"
     */
    public static double extensionFF = 0;

    public static PidfController.PidfCoefficients pitchPidCoefficients =
            new PidfController.PidfCoefficients(1.0, 0, 0.3, 1, 0);
    public static double pitchPidTolerance = 0;
    public final PidfController pitchPid = new PidfController(pitchPidCoefficients);

    public static PidfController.PidfCoefficients extensionPidCoefficients =
            new PidfController.PidfCoefficients(0.07, 0, 0.0009, 1, 0);
    public static double extensionPidTolerance = 0;
    private final PidfController extensionPid = new PidfController(extensionPidCoefficients);

    // States
    private Position armPosition;

    public enum Presets {

    }

    public void resetPitch() {
        // TODO: Use current draw to detect when reached the end
        this.setPitchTarget(0);
    }

    public void resetExtension() {
        // TODO: Use current draw to detect when the end is reached
        this.setExtensionTarget(0);
    }

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
        this.armPosition = getPosition();

        this.pitchPid.setTolerance(pitchPidTolerance);
        this.extensionPid.setTolerance(extensionPidTolerance);
    }

    public Position getPosition() {
        return new Position(getPitchPosition(), getExtensionPosition());
    }

    /**
     * Sets powers to pitch
     * NOTE: Used purely for PID tuning!
     */
    public void setPitchPower(double power) {
        this.armPitchMotor1.setPower(power);
        this.armPitchMotor2.setPower(power);
    }

    /**
     * Sets powers to extension
     * NOTE: Used purely for PID tuning!
     */
    public void setExtensionPower(double power) {
        this.armExtensionMotor1.setPower(power);
        this.armExtensionMotor2.setPower(power);
    }

    /**
     * Sets pitch target
     * NOTE: Does NOT update PID! Use .setPitch to update it in the PID object
     */
    public void setPitchTarget(double target){
        this.armPosition.setPitch(target);
    }

    /**
     * Sets extension target
     * NOTE: Does NOT update PID! Use .setExtension to update it in the PID object
     */
    public void setExtensionTarget(double target){
        this.armPosition.setExtension(target);
    }

    /**
     * Calculates powers for pitch and moves motors
     * Includes adaptive feedforward calculation based on angle & pitch
     */
    public void updatePitch() {
        double desiredPitch = this.armPosition.getPitch();
        this.pitchPid.setTargetPosition(desiredPitch);

        double slope = (value2 - value1) / MAX_EXTENSION;
        double yIntercept = value1;
        // Linear adjustment based on extension
        double calculatedFF = slope * armExtensionEncoder.getCurrentPosition() + yIntercept;

        // Angle Adjusting
        double currentPitch = armPitchEncoder.getCurrentPosition();
        calculatedFF *= Math.cos(currentPitch);
        if (desiredPitch == 0 && pitchPid.atTargetPosition(currentPitch)) {
            calculatedFF = 0;
            // If the arm desired position is flat AND it has reached, there is no need to apply a feedforward
            // This is because there is a hardstop, so it doesn't require any power to keep it up
        }
        double pitchPower = pitchPid.calculatePower(currentPitch, calculatedFF, true);
        this.armPitchMotor1.setPower(pitchPower);
        this.armPitchMotor2.setPower(pitchPower);
    }

    /**
     * Calculates powers for extension and moves motors
     * Includes adaptive feedforward calculation based on pitch
     */
    public void updateExtension() {
        double currentExtension = armExtensionEncoder.getCurrentPosition();
        this.extensionPid.setTargetPosition(this.armPosition.getExtension()); // tells the PID the target position
        double calculatedFF = Math.sin(armPitchEncoder.getCurrentPosition()) * extensionFF;
        if (this.armPosition.getExtension() == 0 && this.extensionPid.atTargetPosition(currentExtension)){
            calculatedFF = 0;
            // If the arm is all the way retracted desired feedforward is just 0 :)
        }
        double extensionPower = extensionPid.calculatePower(currentExtension, calculatedFF);
        this.armExtensionMotor1.setPower(extensionPower);
        this.armExtensionMotor2.setPower(extensionPower);
    }

    /**
     * Increases the pitch by some angle
     * @param angle The angle to increase by, in radians
     */
    public void adjustPitch(double angle) {
        this.armPosition.adjustPitch(angle);
    }

    /**
     * Increases the extension by some angle
     * @param distance The distance to increase by, in inches
     */
    public void adjustExtension(double distance) {
        this.armPosition.adjustExtension(distance);
    }

    /**
     * Returns whether or not the pink arm pids have reached their target position.
     * @return Whether or not the pids have reached
     */
    public boolean atTargetPosition() {
        return this.extensionPid.atTargetPosition(armExtensionEncoder.getCurrentPosition())
                && this.pitchPid.atTargetPosition(armPitchEncoder.getCurrentPosition());
    }

    /**
     * Updates the motors base on the desired pitch and extension.
     * Run this function every loop iteration.
     */
    public void update() {
        this.updatePitch();
        this.updateExtension();
    }

    public double getPitchPosition() {
        return armPitchEncoder.getCurrentPosition();
    }

    public double getExtensionPosition() {
        return armExtensionEncoder.getCurrentPosition();
    }
}

class Position {
    private double pitch;
    private double extension;

    private final static double maxPitch = PinkArm.maxPitch;
    private final static double maxExtension = PinkArm.MAX_EXTENSION;

    /**
     * The position of the arm
     * @param pitch The pitch of the arm, in radians
     * @param extension The extension of the arm, in inches
     */
    public Position(double pitch, double extension) {
        this.pitch = pitch;
        this.extension = extension;
    }

    public double getPitch(){
        return pitch;
    }

    public double getExtension(){
        return extension;
    }

    public void setExtension(double extension) {
        this.extension = Math.min(Math.max(extension, 0), maxExtension);
    }

    public void adjustExtension(double change){
        this.extension += change;
        extension = Math.min(Math.max(extension, 0), maxExtension);
    }

    public void setPitch(double pitch){
        this.pitch = Math.min(Math.max(pitch, 0), maxPitch);
    }

    public void adjustPitch(double change) {
        this.pitch += change;
        this.pitch = Math.min(Math.max(pitch, 0), maxPitch);
    }
}
