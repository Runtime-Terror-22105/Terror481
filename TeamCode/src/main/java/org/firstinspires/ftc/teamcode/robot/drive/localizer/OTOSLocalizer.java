package org.firstinspires.ftc.teamcode.robot.drive.localizer;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math.Pose2d;

public class OTOSLocalizer {
    public static class Parameters {
        // sets offset from center of robot
        public SparkFunOTOS.Pose2D offset;
        public SparkFunOTOS.Pose2D initialPos;

        // first tune ang scalar by spinning robot by multiple rotations (eg. 10) to get an error,
        // then set scalar to inverse of the error (angle wraps from -180 to 180) so for example,
        // if after 10 rotations counterclockwise (positive rotation), the sensor reports -15
        // degrees, the required scalar would be 3600/3585 = 1.004
        // To calibrate the linear scalar, move the robot a known distance and measure the error;
        // do this multiple times at multiple speeds to get an average, then set the linear scalar
        // to the inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        public double linearScalar;
        public double angularScalar;

        public Parameters(@NonNull Pose2d offset, @NonNull Pose2d initialPos, double linearScalar, double angularScalar) {
            this.offset = offset.toOtosPose();
            this.linearScalar = linearScalar;
            this.angularScalar = angularScalar;
            this.initialPos = initialPos.toOtosPose();
        }
    }

    private SparkFunOTOS otos;

    public OTOSLocalizer(SparkFunOTOS otos) {
        this.otos = otos;
    }

    public void initializeOtos(@NonNull Parameters parameters) {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);

        otos.setOffset(parameters.offset);

        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        // 1000 samples * 2.4 ms = 2.4 seconds blocking call
        otos.calibrateImu(1000, true);

        otos.resetTracking(); // resets pos to origin

        // if we want the initial pos to be set to something else
        otos.setPosition(parameters.initialPos);
    }

    public boolean calibrateImu(int numSamples, boolean waitUntilDone) {
        return otos.calibrateImu(numSamples, waitUntilDone);
    }

    public void resetTracking() {
        otos.resetTracking();
    }

    public boolean setLinearScalar(double scalar) {
        return otos.setLinearScalar(scalar);
    }

    public boolean setAngularScalar(double scalar) {
        return otos.setAngularScalar(scalar);
    }

    public Pose2d getAcceleration() {
        return new Pose2d(otos.getAcceleration());
    }

    public Pose2d getVelocity() {
        return new Pose2d(otos.getVelocity());
    }

    public Pose2d getPosition() {
        return new Pose2d(otos.getPosition());
    }

    public Pose2d getPositionStdDev() {
        return new Pose2d(otos.getPositionStdDev());
    }

    public Pose2d getVelocityStdDev() {
        return new Pose2d(otos.getVelocityStdDev());
    }

    public Pose2d getAccelerationStdDev() {
        return new Pose2d(otos.getAccelerationStdDev());
    }
}
