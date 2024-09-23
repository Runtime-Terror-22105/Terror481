package org.firstinspires.ftc.teamcode.robot.drive.swerve;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.controllers.PidfController;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorAxonServo;
import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;

public class SwerveModule {
    // the motors
    private final TerrorMotor driveMotor;
    private final TerrorAxonServo rotationServo;

    public double currAngle;

    // the numbers for calculations
    private final PidfController.PidfCoefficients anglePidCoefficientsCCW;
    private final PidfController.PidfCoefficients anglePidCoefficientsCW;
    public final PidfController anglePid;

//    private final PidfController.PidfCoefficients velPidCoefficients;
//    private final PidfController velPid;

    private double driveVelocity;


    private double drivePower;
    public double servoPower;

    public Coordinate offset;


    /**
     * Creates a new Swerve Module object to be used by the SwerveDrivetrain class.
     *
     * @param driveMotor           The motor that moves the wheel (controls speed)
     * @param rotationServo        The servo that rotates the module (controls rotation)
     */
    public SwerveModule(@NonNull TerrorMotor driveMotor, @NonNull TerrorAxonServo rotationServo,
                        @NonNull SwerveModuleConfig config) {
        this.driveMotor = driveMotor;
        this.rotationServo = rotationServo;
        this.anglePidCoefficientsCCW = config.anglePidfCoefficientsCCW;
        this.anglePidCoefficientsCW = config.anglePidfCoefficientsCW;

        this.anglePid = new PidfController(anglePidCoefficientsCCW);
//        this.rotationServo.setDirection(config.servoEncoderReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        this.rotationServo.setOffset(config.servoEncoderOffset);

        this.offset = config.moduleOffset;

        resetCache();
    }

    /**
     * Set the speed to drive at and the angle for the wheel. PID will be used for the angle.
     * @param speed The power for the speed servo
     * @param goal_angle The desired angle, in radians
     */
    public void updateState(double speed, double goal_angle) {
        //    Setting Variables    //
        double current = this.driveMotor.getCurrent(CurrentUnit.AMPS);
        double idealCurrentDraw = current/9.2*1.47/23.33333333*3.6; // I could simplify this but it's easier to understand the maths this way

        goal_angle = Angle.angleWrap(goal_angle);
        double curr_angle = this.currAngle;

//        boolean wheelFlipped = false;
//        if (Angle.canBeOptimized(curr_angle, goal_angle)) {
//            goal_angle = Angle.angleWrap(goal_angle-Math.PI);
//            wheelFlipped = true;
//        }

        double error = Angle.angleWrap(goal_angle - curr_angle);
        anglePid.setCoefficients(this.getAnglePidCoefficients(error));
        anglePid.setTargetPosition(goal_angle);
        anglePid.calculatePower(curr_angle, 0, 0.04);

        servoPower = (anglePid.atTargetPosition() ? 0 : anglePid.power);

//        if (wheelFlipped) {
//            speed = -speed;
//        }
        this.drivePower = speed;



        //    Moving Stuff    //

        // setting the angle
        this.rotationServo.setPower(servoPower);
        this.driveMotor.setPower(this.drivePower);
//        this.driveMotor.setPower(0.0);
    }

    public PidfController.PidfCoefficients getAnglePidCoefficients(double error) {
        if (error > 0) {
            return anglePidCoefficientsCCW;
        } else {
            return anglePidCoefficientsCW;
        }
    }


    // if the delta between goal is not in +90 or -90 degrees ranged then use other side of wheel to turn there and reverse speed
    /**
     * Get the actual velocity and angle of the wheel.
     * @return The current state of the module.
     */
    public SwerveModuleState getCurrentState() {
        double vel = this.driveVelocity;
        return new SwerveModuleState(vel, this.currAngle);
    }

    public SwerveModuleState getInitialState(){
        return new SwerveModuleState(this.driveVelocity,this.currAngle);
    }


    public void dontMove() {
//        updateState(0, this.currAngle);
        this.rotationServo.setPower(0);
        this.driveMotor.setPower(0);
//        move();
    }

    public double getVelx(){
//        SwerveModuleState current_state=getCurrentState(timeElapsed);
//        double linear_vel=current_state.velocity*(1/28)*(1/8.73)*72*(1/25.4); // converting from encoder tick per ms to wheel rotation per ms to distance covered per ms
        double linear_vel=this.driveVelocity*(1/28)*(1/8.73)*72*(1/25.4)*(1/1000); //wheel rotation per millisecond second
        return Math.cos(this.currAngle)*linear_vel;
    }

    public double getVely(){
//        SwerveModuleState current_state=getCurrentState(timeElapsed);
//        double linear_vel=current_state.velocity*(1/28)*(1/8.73)*72*(1/25.4); // distance covered in mm too
        double linear_vel=this.driveVelocity*(1/28)*(1/8.73)*72*(1/25.4)*(1/1000); //wheel rotation per millisecond second
        return Math.cos(this.currAngle)*linear_vel;
    }

    public void resetCache(){
        // updating velocity
        this.driveVelocity = driveMotor.getVelocity();

        // updating angle
        this.currAngle = Angle.angleWrap(this.rotationServo.getAbsolutePosition());
    }
}
