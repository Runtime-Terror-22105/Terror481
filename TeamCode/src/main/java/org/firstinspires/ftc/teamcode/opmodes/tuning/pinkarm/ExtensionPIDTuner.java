package org.firstinspires.ftc.teamcode.opmodes.tuning.pinkarm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(group="Pink Arm Tuning")
public class ExtensionPIDTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double extensionTarget = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            robot.pinkArm.setExtensionTarget(extensionTarget);
            robot.pinkArm.updateExtension();

            hardware.write();

            double pitch = robot.pinkArm.getPitchPosition();
            robot.telemetry.addData("Current Extension", robot.pinkArm.getExtensionPosition());
            robot.telemetry.addData("Desired Extension", extensionTarget);
            robot.telemetry.addData("Current Pitch (degrees)", Math.toDegrees(pitch));
            robot.telemetry.addData("Current Pitch (radians)", pitch);
            robot.telemetry.update();

        }

//        robot.shutdown();
    }

}
