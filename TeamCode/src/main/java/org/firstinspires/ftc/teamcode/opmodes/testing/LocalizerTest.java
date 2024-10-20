package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@Photon
@TeleOp
public class LocalizerTest extends LinearOpMode {
    public static final double DRIVESPEED = 1.0;
    public static double ROTATION_SPEED = 0.005;
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        double initialX = 0;
        double initialY = 0;
        double initialHeading = 0;

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime drivingTimer = new ElapsedTime();
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            // region Driving
            Coordinate direction = new Coordinate(slr(-gamepad1.left_stick_x), slr(gamepad1.left_stick_y));

            double rotation;
            rotation = Math.pow(slr(-gamepad1.right_stick_x), 11);

            robot.drivetrain.move(
                    direction,
                    rotation,
                    DRIVESPEED
            );



            // endregion driving
            hardware.write();

            SparkFunOTOS.Pose2D curPos = robot.localizer.getPosition();
            telemetry.addData("Current x", curPos.x);
            telemetry.addData("Current y", curPos.y);
            telemetry.addData("Current h", curPos.h);
            telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
            telemetry.update();
            loopTimer.reset();
        }

//        robot.shutdown();
    }
    public double slr(double joystick_value){
        return (Math.pow(joystick_value,3)+joystick_value)/2;
    }
}
