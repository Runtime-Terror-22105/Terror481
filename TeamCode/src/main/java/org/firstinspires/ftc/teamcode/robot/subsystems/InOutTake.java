package org.firstinspires.ftc.teamcode.robot.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorCRServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorServo;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
public class InOutTake {
    public static double SERVO_1_UP = 1.0;
    public static double SERVO_1_DOWN = 0;
    public static double SERVO_2_UP = 1.0;
    public static double SERVO_2_DOWN = 0;

    private final TerrorCRServo wheelServo1;
    private final TerrorCRServo wheelServo2;
    private final TerrorServo pitchServo1;
    private final TerrorServo pitchServo2;

    public InOutTake(@NonNull RobotHardware hardware) {
        this.wheelServo1 = hardware.intakeWheelServo1;
        this.wheelServo2 = hardware.intakeWheelServo2;
        this.pitchServo1 = hardware.intakePitchServo1;
        this.pitchServo2 = hardware.intakePitchServo2;
    }

    public void intake() {
        wheelServo1.setPower(1.0);
        wheelServo2.setPower(1.0);
    }

    public void outtake() {
        wheelServo1.setPower(-1.0);
        wheelServo2.setPower(-1.0);
    }

    public void stop() {
        wheelServo1.setPower(0);
        wheelServo2.setPower(0);
    }

    public void moveUp() {
        pitchServo1.setPosition(SERVO_1_UP);
        pitchServo2.setPosition(SERVO_2_UP);
    }

    public void moveDown() {
        pitchServo1.setPosition(SERVO_1_DOWN);
        pitchServo2.setPosition(SERVO_2_DOWN);
    }
}
