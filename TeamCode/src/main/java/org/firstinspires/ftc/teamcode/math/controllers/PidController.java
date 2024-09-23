package org.firstinspires.ftc.teamcode.math.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PidController {
    private static final double MAX_INTEGRAL = 1e15; // random constant to prevent integral windup, will adjust later

    // pid constants
    private final PidCoefficients pidCoefficients;
    //region pid temp vars
    private double integralSum;
    private double lastError;
    private double error;
    private final ElapsedTime timer = new ElapsedTime();
    //endregion

    //region public variables
    public double power = 0;
    private double targetPosition = 0;
    public boolean reached = false;
    private double threshold = 10;

    //endregion
    public PidController(PidCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
        _resetTempVars();
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    /**
     * NOTE: You must run this function each loop iteration. It will do the PID stuff to calculate
     * the power to be used.
     */
    public void calculatePower(double encoderPosition) {
        error = targetPosition - encoderPosition;

        if (Math.abs(error) >= this.threshold) {
            double derivative = (error - lastError) / timer.seconds();
            integralSum = Math.max(Math.min(integralSum + (error * timer.seconds()), MAX_INTEGRAL), -MAX_INTEGRAL);

            double outUnclamped = (pidCoefficients.Kp * error)
                                + (pidCoefficients.Kd * derivative)
                                + (pidCoefficients.Ki * integralSum);
            power = Math.max(Math.min(outUnclamped, 1.0), -1.0); // out has to be between -1 and 1

            lastError = error;
            timer.reset();
            reached = false;
        } else {
            _resetTempVars();
            reached = true;
            power = 0;
        }
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
        this.timer.reset();
    }

    public static class PidCoefficients {
        public double Kp;
        public double Ki;
        public double Kd;

        public PidCoefficients(double Kp, double Ki, double Kd) {
            this.Kp = Kp;
            this.Ki = Ki;
            this.Kd = Kd;
        }
    }
}
