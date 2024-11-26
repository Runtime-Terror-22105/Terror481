package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Scurve;
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
     * The horizontal extension limit, in inches
     */
    public static final double HORIZONTAL_LIMIT = 400; // TODO: add this value

    /**
     * Max extension at any point, not just horizontal, used to calculate feedforward
     */
    public static final double MAX_EXTENSION = 495;

    /**
     * Max pitch in radians
     */
    public static final double MAX_PITCH = 1.62316;

    /**
     * Value 1 from "PitchFFTuner.java"
     */
    public static double value1 = 0.03;

    /**
     * Value 2 from "PitchFFTuner.java"
     */
    public static double value2 = 0.08;

    // PIDs
    // DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)

    /**
     * Tune this value from "ExtensionPIDTuner.java"
     */
    public static double extensionFF = 0;

    public static PidfController.PidfCoefficients pitchPidCoefficients =
            new PidfController.PidfCoefficients(1.4, 0, 0.02, 1, 0);
    public static double pitchPidTolerance = Math.toRadians(1);
    public final PidfController pitchPid = new PidfController(pitchPidCoefficients);

    public static PidfController.PidfCoefficients extensionPidCoefficients =
            new PidfController.PidfCoefficients(0.05, 0, 0.0009, 1, 0);
    public static double extensionPidTolerance = 12;
    private final PidfController extensionPid = new PidfController(extensionPidCoefficients);

    // States
    private final Position armPosition;

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
        this.armPosition = new Position(0, 0);

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

    public double getExtensionTarget() {
        return this.armPosition.getExtension();
    }

    public double getPitchTarget() {
        return this.armPosition.getPitch();
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
        return extensionAtTargetPosition() && pitchAtTargetPosition();
    }

    /**
     * Whether or not the robot's pitch is at the desired pitch (with some tolerance).
     * @return Whether or not the pitch has reached its target.
     */
    public boolean pitchAtTargetPosition() {
        return this.pitchPid.atTargetPosition(armPitchEncoder.getCurrentPosition());
    }

    /**
     * Whether or not the robot's extension is at the desired extension (with some tolerance).
     * @return Whether or not the extension has reached its target.
     */
    public boolean extensionAtTargetPosition() {
        return this.extensionPid.atTargetPosition(armExtensionEncoder.getCurrentPosition());
    }

    /**
     * Updates the motors base on the desired pitch and extension.
     * Run this function every loop iteration.
     */
    public void update() {
        // TODO: Make a seperate function to only update extension target when pivot is like 80% done
        this.updatePitch();
        this.updateExtension();
    }

    public double getPitchPosition() {
        return armPitchEncoder.getCurrentPosition();
    }

    public double getExtensionPosition() {
        return armExtensionEncoder.getCurrentPosition();
    }


    public void move(double goal){
        double curr=getPitchPosition();
        double direction=Math.signum(goal-curr);
        Scurve pinkcurve=new Scurve(0,Math.abs(goal-curr));
        double t=0; //reverse solve for the time and get the velocity

    }
}

class Position {
    private double pitch;
    private double extension;

    private final static double MAX_PITCH = PinkArm.MAX_PITCH;
    private final static double MAX_EXTENSION = PinkArm.MAX_EXTENSION;

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

    public void setPitch(double pitch){
        this.pitch = pitch;
        controlPitch();
    }

    public void setExtension(double extension){
        this.extension = extension;
        controlExtension();
    }

    public void adjustPitch(double change){
        this.pitch += change;
        controlPitch();
    }

    public void adjustExtension(double change){
        this.extension += change;
        controlExtension();
    }

    private void controlExtension() {
        if(extension < 0) extension = 0;
        if(extension > MAX_EXTENSION) extension = MAX_EXTENSION;

        if (pitch < Math.toRadians(5) && extension > PinkArm.HORIZONTAL_LIMIT) {
            extension = PinkArm.HORIZONTAL_LIMIT;
        }
    }
    private void controlPitch() {
        if(pitch < 0) pitch = 0;
        if(pitch > MAX_PITCH) pitch = MAX_PITCH;
    }
}