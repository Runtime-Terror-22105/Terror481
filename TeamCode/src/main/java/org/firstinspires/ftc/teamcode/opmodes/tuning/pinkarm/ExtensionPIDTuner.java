package org.firstinspires.ftc.teamcode.opmodes.tuning.pinkarm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@Photon
@TeleOp(group="Pink Arm Tuning")
public class ExtensionPIDTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();
    private FtcDashboard dashboard;

    public static double extensionTarget = 0.0;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        robot.init(this, hardware, telemetry);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            robot.pinkArm.setExtensionTarget(extensionTarget);
            robot.pinkArm.updateExtension();
            robot.pinkArm.moveArmToPosition();


            telemetry.addData("Current Extension: ", robot.pinkArm.getExtensionPosition());
            telemetry.addData("Current Pitch: ", robot.pinkArm.getPitchPosition());
            telemetry.update();

        }

//        robot.shutdown();
    }

}
