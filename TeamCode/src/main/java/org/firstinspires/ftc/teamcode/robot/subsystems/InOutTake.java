package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class InOutTake {
    public static double SERVO_UP = 0.0;
    public static double SERVO_MIDDLE = 0.35;
    public static double SERVO_DOWN = 0.8;

    private final TerrorCRServo wheelServoLeft;
    private final TerrorCRServo wheelServoRight;
    private final TerrorServo pitchServo;

    public InOutTake(@NonNull RobotHardware hardware) {
        this.wheelServoLeft = hardware.intakeWheelServoLeft;
        this.wheelServoRight = hardware.intakeWheelServoRight;
        this.pitchServo = hardware.intakePitchServo;
    }

    public void intake() {
        wheelServoLeft.setPower(0.5);
        wheelServoRight.setPower(-0.5);
    }

    public void outtake() {
        wheelServoLeft.setPower(-0.5);
        wheelServoRight.setPower(0.5);
    }

    public void stop() {
        wheelServoLeft.setPower(0);
        wheelServoRight.setPower(0);
    }

    public void moveUp() {
        pitchServo.setPosition(SERVO_UP);
    }

    public void moveDown() {
        // make this work
        pitchServo.setPosition(SERVO_DOWN);
    }

    public void moveMiddle() {
        pitchServo.setPosition(SERVO_MIDDLE);
    }

    public void adjustPitch(double val) {
        double lastPos = pitchServo.getSetPosition();
        this.pitchServo.setPosition(lastPos + val);
    }
}
