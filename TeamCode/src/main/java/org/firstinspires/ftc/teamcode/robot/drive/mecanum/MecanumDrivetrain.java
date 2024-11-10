package org.firstinspires.ftc.teamcode.robot.drive.mecanum;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;

public class MecanumDrivetrain implements Drivetrain {
    private final TerrorMotor motorRearLeft;
    private final TerrorMotor motorFrontLeft;
    private final TerrorMotor motorRearRight;
    private final TerrorMotor motorFrontRight;

    /**
     * Initializes a swerve drivetrain.
     * @param motorRearLeft Self explanatory
     * @param motorFrontLeft Self explanatory
     * @param motorRearRight Self explanatory
     * @param motorFrontRight Self explanatory
     */
    public MecanumDrivetrain(TerrorMotor motorRearLeft, TerrorMotor motorFrontLeft,
                             TerrorMotor motorRearRight, TerrorMotor motorFrontRight) {
        this.motorRearLeft = motorRearLeft;
        this.motorFrontLeft = motorFrontLeft;
        this.motorRearRight = motorRearRight;
        this.motorFrontRight = motorFrontRight;

        this.motorRearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorRearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Move the robot by some amount
     * @param velocity Movement on x and y
     * @param rotation Counterclockwise rotation (NOTE: gm0 mecanum is clockwise, this is diff)
     */
    @Override
    public void move(@NonNull Coordinate velocity, double rotation) {
        this.motorFrontLeft.setPower(velocity.y + velocity.x - rotation);
        this.motorRearLeft.setPower(velocity.y - velocity.x - rotation);
        this.motorFrontRight.setPower(velocity.y - velocity.x + rotation);
        this.motorRearRight.setPower(velocity.y + velocity.x + rotation);
    }
}
