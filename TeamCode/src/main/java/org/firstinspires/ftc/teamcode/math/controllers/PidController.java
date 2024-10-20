package org.firstinspires.ftc.teamcode.math.controllers;

import com.qualcomm.robotcore.util.Range;

public class PidController {
    private static final double MAX_INTEGRAL = 1e15; // random constant to prevent integral windup, will adjust later

    // pid constants
    private final PidCoefficients pidCoefficients;
    //region pid temp vars
    private double integralSum;
    private double lastError;
    private double error;
    private double lastTimeStamp;
    //endregion

    //region public variables
    private double targetPosition = 0;
    private double tolerance = 10;

    //endregion
    public PidController(PidCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
        _resetTempVars();
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculateError(double encoderPosition) {
        return this.targetPosition - encoderPosition;
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     */
    public double calculatePower(double currentPosition) {
        this.error = calculateError(currentPosition);

        double timestamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = timestamp;
        double period = Math.max(timestamp - lastTimeStamp, 1E-5);
        lastTimeStamp = timestamp;

        double derivative = (error - lastError) / period;
        this.integralSum = Math.max(Math.min(integralSum + (error * period), MAX_INTEGRAL), -MAX_INTEGRAL);

        double power = (pidCoefficients.Kp * error)
                + (pidCoefficients.Kd * derivative)
                + (pidCoefficients.Ki * integralSum);
        this.lastError = error;

        if (Math.abs(error) > tolerance) {
            power += power >= 0 ? pidCoefficients.Kstatic : -pidCoefficients.Kstatic;
        }

        power = Range.clip(power, -1, 1);
        if (Double.isNaN(power)) power = 0;

        return power;
    }

    // region moving

    /**
     * Increase/decrease the target position of the PID by some amount of counts. This is a
     * RELATIVE move unlike setTargetPosition().
     * @param moveAmt The amount of clicks to increase/decrease the position by.
     */
    public void move(double moveAmt) {
        double newTargetPos = this.targetPosition + moveAmt;
        this.setTargetPosition(newTargetPos);
    }

    /**
     * Sets the target position of the slides.
     * @param targetPosition The new target position for the slides.
     */
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public boolean atTargetPosition(double currentPosition) {
        return calculateError(currentPosition) <= this.tolerance;
    }

    // endregion
    /**
     * Gets the current target position of the PID.
     * @return targetPosition - The target position.
     */
    public double getTargetPosition() {
        return this.targetPosition;
    }

    private void _resetTempVars() {
        this.integralSum = 0;
        this.lastError = 0;
        this.error = 0;
        this.lastTimeStamp = 0;
    }

    public static class PidCoefficients {
        public double Kp;
        public double Ki;
        public double Kd;
        public double Kstatic;

        public PidCoefficients(double Kp, double Ki, double Kd, double Kstatic) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
            this.Kstatic = Kstatic;
        }

        public PidCoefficients(double Kp, double Ki, double Kd) {
            this(Kp, Ki, Kd, 0);
        }
    }
}
