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

    public void setState(State state) {
        this.state = state;
    }

    public void setPitchPower(double power) {
        this.armPitchMotor1.setPower(power);
        this.armPitchMotor2.setPower(power);
    }

    public boolean stateIs(State other) {
        return this.state.equals(other);
    }

    /**
     * Sets the desired pitch and extension of the pink arm
     * @param position The position of the arm
     */
    private void setPidTargets() {
        this.pitchPid.setTargetPosition(this.armPosition.pitch);
        this.extensionPid.setTargetPosition(this.armPosition.extension);
    }

    /**
     * Moves the motors for the pitch
     */
    private void updatePitch() {
        double pitchPower = pitchPid.calculatePower(armPitchEncoder.getCurrentPosition(), 0);
        this.armPitchMotor1.setPower(pitchPower);
        this.armPitchMotor2.setPower(pitchPower);
    }

    /**
     * Update the motors for the extension
     */
    private void updateExtension() {
        double extensionPower = extensionPid.calculatePower(armExtensionEncoder.getCurrentPosition());
        this.armExtensionMotor1.setPower(extensionPower);
        this.armExtensionMotor2.setPower(extensionPower);
    }

    /**
     * Increases the pitch by some angle
     * @param angle The angle to increase by, in radians
     */
    public void adjustPitch(double angle) {
        this.armPosition.pitch += angle;
        this.state = State.MANUAL;
    }

    /**
     * Increases the extension by some angle
     * @param distance The distance to increase by, in inches
     */
    public void adjustExtension(double distance) {
        this.armPosition.extension += distance;
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
            this.armPosition = state.getPosition();
        }
        this.setPidTargets();

        this.moveArmToPosition();
    }

    private void moveArmToPosition() {
        this.updatePitch();
        this.updateExtension();
    }

    public static class Position {
        public double pitch;
        public double extension;

        /**
         * The position of the arm
         * @param pitch The pitch of the arm, in radians
         * @param extension The extension of the arm, in inches
         */
        public Position(double pitch, double extension) {
            this.pitch = pitch;
            this.extension = extension;
        }
    }
}
