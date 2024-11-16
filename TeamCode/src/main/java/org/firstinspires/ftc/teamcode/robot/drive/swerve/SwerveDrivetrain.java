//package org.firstinspires.ftc.teamcode.robot.drive.swerve;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.math.Coordinate;
//import org.firstinspires.ftc.teamcode.math.NumCompare;
//import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
//import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorAxonServo;
//import org.firstinspires.ftc.teamcode.robot.hardware.motors.TerrorMotor;
//
///**
// * The SwerveDriveTrain class provides simple methods to easily control a swerve drivetrain in FTC.
// * The goal is that this is simply a "black box" that will allow coders to focus on other things.
// * 4 PID controllers total are used for the rotation of each module.
// */
//@Config
//public class SwerveDrivetrain implements Drivetrain {
//    private static final double ZERO_POWER_THRESHOLD = 0.005;
//    //region    Swerve Modules
//    public final SwerveModule moduleRearLeft;
//    public final SwerveModule moduleFrontLeft;
//    public final SwerveModule moduleRearRight;
//    public final SwerveModule moduleFrontRight;
//    //endregion Swerve Modules
//
//    private final Telemetry telemetry;
//
//    // public vars
//    public double RearLeftPower;
//    public double frontLeftPower;
//    public double RearRightPower;
//    public double frontRightPower;
//    public SwerveDrivetrainConfig config;
//    public SwerveModuleConfig[] module_configs = new SwerveModuleConfig[4];
//
//    /**
//     * Initializes a swerve drivetrain.
//     * @param telemetry The telemetry
//     * @param motorRearLeft Self explanatory
//     * @param servoRearLeft Self explanatory
//     * @param moduleRearLeftConfig Self explanatory
//     * @param motorFrontLeft Self explanatory
//     * @param servoFrontLeft Self explanatory
//     * @param moduleFrontLeftConfig Self explanatory
//     * @param motorRearRight Self explanatory
//     * @param servoRearRight Self explanatory
//     * @param moduleRearRightConfig Self explanatory
//     * @param motorFrontRight Self explanatory
//     * @param servoFrontRight Self explanatory
//     * @param moduleFrontRightConfig Self explanatory
//     */
//    public SwerveDrivetrain(Telemetry telemetry, SwerveDrivetrainConfig swerveConfig,
//                            TerrorMotor motorRearLeft, TerrorAxonServo servoRearLeft, SwerveModuleConfig moduleRearLeftConfig,
//                            TerrorMotor motorFrontLeft, TerrorAxonServo servoFrontLeft, SwerveModuleConfig moduleFrontLeftConfig,
//                            TerrorMotor motorRearRight, TerrorAxonServo servoRearRight, SwerveModuleConfig moduleRearRightConfig,
//                            TerrorMotor motorFrontRight, TerrorAxonServo servoFrontRight, SwerveModuleConfig moduleFrontRightConfig) {
//        this.telemetry = telemetry;
//        this.config = swerveConfig;
//        this.moduleRearLeft   = new SwerveModule(motorRearLeft, servoRearLeft, moduleRearLeftConfig);
//        this.moduleFrontLeft  = new SwerveModule(motorFrontLeft, servoFrontLeft, moduleFrontLeftConfig);
//        this.moduleRearRight  = new SwerveModule(motorRearRight, servoRearRight, moduleRearRightConfig);
//        this.moduleFrontRight = new SwerveModule(motorFrontRight, servoFrontRight, moduleFrontRightConfig);
//
//
//        this.module_configs[0] = moduleFrontLeftConfig;
//        this.module_configs[1] = moduleRearLeftConfig;
//        this.module_configs[2] = moduleFrontRightConfig;
//        this.module_configs[3] = moduleRearRightConfig;
//        // set RUN_WITHOUT_ENCODER and reset the encoders here
//    }
//
//    /**
//     * Sets a new target position
//     * @param velocity The x and y
//     * @param rotation The rotation
//     */
//    @Override
//    public void move(@NonNull Coordinate velocity, double rotation) { // second order kinematics coming soon tm
//        if (Math.abs(velocity.x) < ZERO_POWER_THRESHOLD && Math.abs(velocity.y) < ZERO_POWER_THRESHOLD && Math.abs(rotation) < ZERO_POWER_THRESHOLD) {
//            this.moduleFrontRight.dontMove();
//            this.moduleFrontLeft.dontMove();
//            this.moduleRearLeft.dontMove();
//            this.moduleRearRight.dontMove();
//            return;
//        }
//
//        double A = velocity.x - rotation*(this.config.wheelBase /this.config.R); // L/R
//        double B = velocity.x + rotation*(this.config.wheelBase /this.config.R); // L/R
//        double C = velocity.y - rotation*(this.config.trackWidth/this.config.R); // W/R
//        double D = velocity.y + rotation*(this.config.trackWidth/this.config.R); // W/R
//
//        // divide by the max val in order to make sure the wheels go in the correct direction and
//        // don't have their powers clipped
//        double wheelFrontRightPower = Math.sqrt(B*B + C*C);
//        double wheelFrontLeftPower = Math.sqrt(B*B + D*D);
//        double wheelRearLeftPower = Math.sqrt(A*A + D*D);
//        double wheelRearRightPower = Math.sqrt(A*A + C*C);
//        double scale = NumCompare.max(Math.abs(wheelRearRightPower), Math.abs(wheelRearLeftPower),
//                Math.abs(wheelFrontLeftPower), Math.abs(wheelFrontRightPower), 1);
//        if(scale == 0) {
//            scale = 1;
//        }
//        this.moduleFrontRight.updateState(wheelFrontRightPower/scale, (Math.PI-Math.atan2(B, C)));
//        this.moduleFrontLeft.updateState(wheelFrontLeftPower/scale, (Math.PI-Math.atan2(B, D)));
//        this.moduleRearLeft.updateState(wheelRearLeftPower/scale, (Math.PI-Math.atan2(A, D)));
//        this.moduleRearRight.updateState(wheelRearRightPower/scale, (Math.PI-Math.atan2(A, C)));
//    }
//
//    public SwerveModule[] getModules(){
//        return new SwerveModule[] {
//                moduleFrontLeft,
//                moduleRearLeft,
//                moduleFrontRight,
//                moduleRearRight
//        };
//    }
//    public SwerveModuleConfig[] getConfigs(){
//        return module_configs;
//    }
//
//    public Coordinate[] getOffsets(){
//        return new Coordinate[] {
//                moduleFrontLeft.offset,
//                moduleRearLeft.offset,
//                moduleFrontRight.offset,
//                moduleRearRight.offset
//        };
//    }
//
//    public SwerveModuleState[] getStates(){
//        return new SwerveModuleState[] {
//                moduleFrontLeft.getCurrentState(),
//                moduleRearLeft.getCurrentState(),
//                moduleFrontRight.getCurrentState(),
//                moduleRearRight.getCurrentState()
//        };
//    }
//
//    public double getVelX(){
//        return (moduleFrontLeft.getVelx()+ moduleFrontRight.getVelx()+ moduleRearLeft.getVelx()+ moduleRearRight.getVelx())/4;
//    }
//    public double getVelY(){
//        return (moduleFrontLeft.getVely()+ moduleFrontRight.getVely()+ moduleRearLeft.getVely()+ moduleRearRight.getVely())/4;
//    }
//
//    public double getAvgLinearVel(){
//
//        return 0.0;
//    }
//
//    public void resetCache(){// updating all of the cached velocities and angles of each module
//        moduleFrontLeft.resetCache();
//        moduleFrontRight.resetCache();
//        moduleRearLeft.resetCache();
//        moduleRearRight.resetCache();
//    }
//}
