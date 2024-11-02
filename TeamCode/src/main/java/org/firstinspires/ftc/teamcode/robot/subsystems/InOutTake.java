package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.SampleColor;
import org.firstinspires.ftc.teamcode.robot.hardware.sensors.TerrorColorRangeFinder;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

/**
 * The in/out take mechanism
 */
public class InOutTake implements Subsystem {
    // hardware
    private final TerrorServo pitchServo1;
    private final TerrorServo pitchServo2;
    private final TerrorCRServo rotationServoLeft;
    private final TerrorCRServo rotationServoRight;
    private final TerrorColorRangeFinder colorSensor;

    // other vars
    private SampleColor[] targetColors;
    private double pitchAngle = 0;
    private enum State {
        INTAKING,
        OUTTAKING,
        STOPPED
    }
    private State currentState;


    /**
     * Creates a new InOutTake class
     * @param hardware The robot's hardware
     */
    public InOutTake(@NonNull RobotHardware hardware) {
        this.pitchServo1 = hardware.wheelPitchServo1;
        this.pitchServo2 = hardware.wheelPitchServo2;
        this.rotationServoLeft = hardware.wheelRotationServoLeft;
        this.rotationServoRight = hardware.wheelRotationServoRight;
        this.colorSensor = hardware.wheelColorSensor;
    }

    /**
     * Set the angle of the intake/outtake mechanism
     * @param angle The desired angle
     */
    public void setAngle(double angle) {
        this.pitchAngle = angle;
    }

    /**
     * Sets power to the intake/outtake wheels
     * @param power The power
     */
    private void setPower(double power) {
        this.rotationServoLeft.setPower(power);
        this.rotationServoRight.setPower(power);
    }

    /**
     * Sets the current state to intaking
     * @param targetColors The color(s) that can be picked up
     */
    public void intake(SampleColor[] targetColors) {
        this.targetColors = targetColors;
        this.currentState = State.INTAKING;
    }

    /**
     * Sets the current state to outtaking
     */
    public void outtake() {
        this.currentState = State.OUTTAKING;
    }

    /**
     * Moves the parts of the intake/outtake based on the current state.
     * Call this every loop iteration.
     */
    public void update() {
        switch (this.currentState) {
            case INTAKING:
                if (this.colorSensor.matchesColor(this.targetColors)) {
                    this.currentState = State.STOPPED;
                    break;
                }
                this.setPower(1.0);
                break;
            case OUTTAKING:
                if (this.colorSensor.getColor().equals(SampleColor.NONE)) {
                    this.currentState = State.STOPPED;
                    break;
                }
                this.setPower(-1.0);
                break;
            case STOPPED:
                this.setPower(0.0);
                break;
        }

        this.pitchServo1.setPosition(this.pitchAngle);
        this.pitchServo2.setPosition(this.pitchAngle);
    }
}
