package org.firstinspires.ftc.teamcode.robot.hardware.motors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.hardware.TerrorWritingDevice;

import java.util.Arrays;

/**
 * A wrapper class for a PhotonDcMotor with additional features like command queuing,
 * power threshold handling, and motor enabling/disabling.
 * This class implements the TerrorHardwareDevice interface.
 */
public class TerrorMotor implements TerrorWritingDevice {
//    private boolean motorEnabled;
    private final DcMotorEx motor;
    private final double powerThreshold;

    private double motorPower;
    private double lastPower;
    private double motorVel;
    private DcMotor.RunMode mode;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    private final MotorCommand[] commands = new MotorCommand[4];

    public double getSetPower() {
        return motorPower;
    }

    /**
     * Enum representing various motor commands that can be queued and executed.
     */
    private enum MotorCommand {
        SET_VEL,
        SET_POWER,
        SET_MOTOR_ENABLE,
        SET_ZERO_POWER_BEHAVIOR,
        SET_MODE,
        NONE
    }

    /**
     * Creates a new TerrorMotor instance.
     *
     * @param motor          The underlying PhotonDcMotor that this class wraps around.
     * @param powerThreshold The threshold used to prevent unnecessary motor power updates.
     */
    public TerrorMotor(@NonNull DcMotorEx motor, double powerThreshold) {
        this.resetCommands();
        this.powerThreshold = powerThreshold;
        this.motor = motor;
//        setMotorEnable();
        this.lastPower = -100; // since it is outside of -1 to 1 range, it will force ignore cache
        this.motorPower = 0;
        this.motorVel = 0;
//        this.motorEnabled = true;
        this.mode = this.motor.getMode();
    }

//    /**
//     * Enables the motor. If the motor is already enabled, this method does nothing.
//     */
//    synchronized public void setMotorEnable() {
//        if (!motorEnabled) {
//            this.motorEnabled = true;
//            this.pushCommand(MotorCommand.SET_MOTOR_ENABLE);
//        }
//    }
//
//    /**
//     * Disables the motor. If the motor is already disabled, this method does nothing.
//     */
//    synchronized public void setMotorDisable() {
//        if (this.motorEnabled) {
//            this.motorEnabled = false;
//            this.pushCommand(MotorCommand.SET_MOTOR_ENABLE);
//        }
//    }

    /**
     * Sets the zero power behavior of the motor.
     *
     * @param zeroPowerBehavior The desired zero power behavior (e.g., BRAKE, FLOAT).
     */
    synchronized public void setZeroPowerBehavior(@NonNull DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (zeroPowerBehavior != this.zeroPowerBehavior) {
            this.pushCommand(MotorCommand.SET_ZERO_POWER_BEHAVIOR);
            this.zeroPowerBehavior = zeroPowerBehavior;
        }
    }

    /**
     * Sets the run mode of the motor.
     *
     * @param mode The desired run mode (e.g., RUN_USING_ENCODER, RUN_WITHOUT_ENCODER).
     */
    synchronized public void setMode(@NonNull DcMotor.RunMode mode) {
        if (!this.mode.equals(mode)) {
            this.mode = mode;
            this.pushCommand(MotorCommand.SET_MODE);
        }
    }

    /**
     * Sets the velocity of the motor in a given angular unit.
     *
     * @param angularRate The desired velocity.
     * @param unit        The unit of angular measurement (e.g., DEGREES, RADIANS).
     */
    synchronized public void setVelocity(double angularRate, @NonNull AngleUnit unit) {
        if (unit.equals(AngleUnit.DEGREES)) {
            angularRate = Math.toRadians(angularRate);
        }
        this.motorVel = angularRate;
        this.pushCommand(MotorCommand.SET_VEL);
    }

    /**
     * Sets the power of the motor.
     *
     * @param power The desired power value (range: -1.0 to 1.0).
     */
    synchronized public void setPower(double power) {
        this.pushCommand(MotorCommand.SET_POWER);
        this.motorPower = power;
    }

    /**
     * Sets the direction of the motor.
     *
     * @param direction The desired direction (e.g., FORWARD, REVERSE).
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        this.motor.setDirection(direction);
    }

    /**
     * Retrieves the current direction of the motor.
     *
     * @return The current direction of the motor.
     */
    public DcMotorSimple.Direction getDirection() {
        return this.motor.getDirection();
    }

//    /**
//     * Checks if the motor is currently enabled.
//     *
//     * @return true if the motor is enabled, false otherwise.
//     */
//    public boolean isMotorEnabled() {
//        return this.motor.isMotorEnabled();
//    }

    /**
     * Retrieves the current position of the motor's encoder.
     *
     * @return The current encoder position.
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Retrieves the current draw of the motor in a given unit.
     *
     * @param unit The desired unit of current (e.g., AMPS).
     * @return The current draw of the motor.
     */
    public double getCurrent(CurrentUnit unit) {
        return motor.getCurrent(unit);
    }

    /**
     * Retrieves the current velocity of the motor.
     *
     * @return The current velocity of the motor.
     */
    public double getVelocity() {
        return this.motor.getVelocity();
    }

    /**
     * Retrieves the current run mode of the motor.
     *
     * @return The current run mode of the motor.
     */
    public DcMotor.RunMode getMode() {
        return this.motor.getMode();
    }

    /**
     * Executes the queued motor commands. This should be called each loop iteration
     * to apply any changes in motor settings such as power, velocity, or mode.
     */
    synchronized public void write() {
        for (MotorCommand command : commands) {
            if (command == MotorCommand.NONE) break; // since commands are in order, we can stop

            switch (command) {
                case SET_POWER:
                    // if motor power is 0 (to enforce zero power behav) or change > threshold
                    if (this.motorPower == 0 || Math.abs(this.motorPower - this.lastPower) > this.powerThreshold) {
                        this.lastPower = this.motorPower;
                        this.motor.setPower(motorPower);
                    }
                    break;
                case SET_VEL:
                    this.motor.setVelocity(motorVel, AngleUnit.RADIANS);
                    break;
//                case SET_MOTOR_ENABLE:
//                    if (motorEnabled) {
//                        this.motor.setMotorEnable();
//                    } else {
//                        this.motor.setMotorDisable();
//                    }
//                    break;
                case SET_ZERO_POWER_BEHAVIOR:
                    this.motor.setZeroPowerBehavior(zeroPowerBehavior);
                    break;
                case SET_MODE:
                    this.motor.setMode(mode);
                    break;
            }
        }
        resetCommands();
    }

    /**
     * Pushes a command to the queue. If a command already exists in the queue,
     * it will be overwritten with the new command.
     *
     * @param command The command to push to the queue.
     * @return true if the command was successfully pushed, false if the command queue is full.
     */
    private boolean pushCommand(MotorCommand command) {
        int modifyPosition = -1;
        for (int i = 0; i < this.commands.length; i++) {
            if (this.commands[i].equals(command)) {
                modifyPosition = i;
                break;
            } else if (this.commands[i] == MotorCommand.NONE) {
                modifyPosition = i;
                break;
            }
        }
        if (modifyPosition >= 0) {
            this.commands[modifyPosition] = command;
            return true;
        }
        return false; // if the maximum number of commands reached
    }

    /**
     * Resets the command queue, clearing all pending commands.
     */
    private void resetCommands() {
        Arrays.fill(this.commands, MotorCommand.NONE);
    }
}
