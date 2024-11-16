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
public class PinkArm implements Subsystem {
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
    public static final double maxExtension = 50;

    /**
     * Max pitch in radians
     */
    public static final double maxPitch = 1.62316;

    /**
     * Value 1 from "PitchFFTuner.java"
     */
    public static double value1 = 0;

    /**
     * Value 2 from "PitchFFTuner.java"
     */
    public static double value2 = 0;

    // PIDs
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)
    // TODO: VERY URGENT: DO *NOT* CHANGE KV OR KSTATIC!!!! (And probably not Ki)

    /**
     * Tune this value from "ExtensionPIDTuner.java"
     */
    public static double extensionFF = 0;

    public static PidfController.PidfCoefficients pitchPidCoefficients =
            new PidfController.PidfCoefficients(0, 0, 0, 1, 0);
    private final PidfController pitchPid = new PidfController(pitchPidCoefficients);
    public static PidfController.PidfCoefficients extensionPidCoefficients =
            new PidfController.PidfCoefficients(0, 0, 0, 1, 0);
    private final PidfController extensionPid = new PidfController(extensionPidCoefficients);

    // States
    private State state = State.MANUAL;
    private Position armPosition = state.getPosition();

    public boolean isHanging() {
        return this.stateIs(State.HANG_1) || this.stateIs(State.HANG_2);
    }

    public enum State {
        HIGH_BASKET(new Position(29, 12)), // locked to position of outtaking to high basket
        LOW_BASKET(new Position(23, 42)), // locked to position of outtaking to low basket
        SUBMERSIBLE(new Position(75, 45)), // locked to position of intaking from submersible
        TAPE(new Position(35, 86)), // locked to position of intaking from tape
        HANG_1(new Position(35, 86)), // hanging part 1
        HANG_2(new Position(35, 86)), // hanging part 2
        MANUAL(new Position(0, 0)); // not locked to any position

        private final Position position;

        State(Position position) {
            this.position = position;
        }

        public Position getPosition() {
            return position;
        }
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
    }

    /**
     * Sets the desired state of the pink arm
     */
    public void setState(State state) {
        this.state = state;
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
     * Checks if oen state equals the current state
     */
    public boolean stateIs(State other) {
        return this.state.equals(other);
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
        double slope = (value2 - value1)/maxExtension;
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
        double pitchPower = pitchPid.calculatePower(currentPitch, calculatedFF);
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
        double calculatedFF = Math.cos(armPitchEncoder.getCurrentPosition()) * extensionFF;
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
        this.state = State.MANUAL;
    }

    /**
     * Increases the extension by some angle
     * @param distance The distance to increase by, in inches
     */
    public void adjustExtension(double distance) {
        this.armPosition.adjustExtension(distance);
        this.state = State.MANUAL;
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
        if (!state.equals(State.MANUAL)) {
            this.armPosition = state.getPosition(); // set targets
        }

        this.moveArmToPosition(); // move motors. this calls both .updatePitch and .updateExtension
    }

    public double getPitchPosition(){
        return armPitchEncoder.getCurrentPosition();
    }

    public double getExtensionPosition(){
        return armExtensionEncoder.getCurrentPosition();
    }

    public void moveArmToPosition() {
        this.updatePitch();
        this.updateExtension();
    }
}
class Position {
    private double pitch;
    private double extension;

    private final static double maxPitch = PinkArm.maxPitch;
    private final static double maxExtension = PinkArm.maxExtension;

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

    private void controlExtension(){
        if(extension < 0) extension = 0;
        if(extension > maxExtension) extension = maxExtension;

        // TODO: Add 42 in max horizontal limit
    }
    private void controlPitch(){
        if(pitch < 0) pitch = 0;
        if(pitch > maxPitch) extension = maxPitch;
    }
}
