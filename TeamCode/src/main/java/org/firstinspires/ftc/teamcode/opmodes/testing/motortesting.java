package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.Coordinate;
import org.firstinspires.ftc.teamcode.math.Pose2d;
import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp
public class motortesting extends LinearOpMode {
    public static final double DRIVESPEED = 1.0;
    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            // region Driving
            robot.hardware.armPitchMotor1.setPower(0.05);
            // endregion driving
            hardware.write();

            Pose2d curPos = robot.localizer.getPosition();
            robot.telemetry.addData("Current x", curPos.x);
            robot.telemetry.addData("Current y", curPos.y);
            robot.telemetry.addData("Current h", curPos.heading);
            robot.telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
            robot.telemetry.addData("Loop time (hz)", 1000/loopTimer.milliseconds());
            robot.telemetry.update();
            loopTimer.reset();
        }
    }
}

