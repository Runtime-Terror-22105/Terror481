package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@Photon
@TeleOp
public class FieldCentricTeleOp extends LinearOpMode {
    public static final double DRIVESPEED = 1.0;
    public static double ROTATION_SPEED = 0.005;
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private FtcDashboard dashboard;

//    private double headingLockAngle = 0; // the angle to lock the heading to, in radians
//    public static PidfController.PidfCoefficients heading_coeff = new PidfController.PidfCoefficients(0.65, 0, 0.065, 0, 0.05);
//    public static PidfController heading = new PidfController(heading_coeff);

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        double initialX = 0;
        double initialY = 0;
        double initialHeading = 0;
//        SwerveLocalizer localizer = new SwerveLocalizer(
//            initialX,
//            initialY,
//            initialHeading,
//            robot
//        );

        waitForStart();
//        robot.imu.setPollTime(2);

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime drivingTimer = new ElapsedTime();
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }
//            robot.drivetrain.resetCache();

            // region Driving
//            localizer.update();
//            double robotAngle = robot.imu.readLastAngle().yaw;

            Coordinate direction = new Coordinate(slr(-gamepad1.left_stick_x), slr(gamepad1.left_stick_y));
//            direction.rotate(-robotAngle);

            double rotation;
//            if (gamepad1.y) { // no heading lock
                rotation = Math.pow(slr(-gamepad1.right_stick_x), 11);
//            } else { // heading lock
//                this.headingLockAngle += Math.pow(slr(-gamepad1.right_stick_x), 11) * ROTATION_SPEED * drivingTimer.milliseconds();
//                this.headingLockAngle= Angle.angleWrap(this.headingLockAngle);
//                telemetry.addData("Heading Current Angle in joystick", this.headingLockAngle);
//                telemetry.update();
//                heading.setTargetPosition(this.headingLockAngle);
//
//                heading.calculatePower(robotAngle, 0, Math.toRadians(1.25));
//                rotation = heading.power;
//                drivingTimer.reset();
//            }

            robot.drivetrain.move(
                    direction,
                    rotation,
                    DRIVESPEED
            );



            // endregion driving

            telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
//            telemetry.addData("Robot angle (degrees)", Math.toDegrees(robotAngle));
            telemetry.update();
            loopTimer.reset();
        }

//        robot.shutdown();
    }
    public double slr(double joystick_value){
        return (Math.pow(joystick_value,3)+joystick_value)/2;
    }
}