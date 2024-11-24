package org.firstinspires.ftc.teamcode.opmodes.tuning.pinkarm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@TeleOp(group="Pink Arm Tuning")
public class PitchVelPIDTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

    public static double pitchVelTarget = 0.0;
    public static double extensionTarget = 0.0;

//    public static PidController.PidCoefficients pitchVelPidCoeff = new PidController.PidCoefficients(0, 0, 0, 0);
//    private final PidController pitchVelPid = new PidController(pitchVelPidCoeff);

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

            double currentPitchVel = robot.pinkArm.getPitchVelocity();

            robot.pinkArm.setExtensionTarget(extensionTarget);
            robot.pinkArm.setPitchTarget(pitchVelTarget);
            robot.pinkArm.updateExtension();
            robot.pinkArm.updatePitchVel(currentPitchVel);

            hardware.write();

            double error = robot.pinkArm.pitchPid.calculateError(currentPitchVel);
            boolean reached = robot.pinkArm.pitchPid.atTargetPosition(currentPitchVel);

            robot.telemetry.addData("Pitch error", error);
            robot.telemetry.addData("Pitch at target position", reached);
            robot.telemetry.addData("Current Pitch Velocity", currentPitchVel);
            robot.telemetry.addData("Desired Pitch Velocity", pitchVelTarget);
            robot.telemetry.addData("Current Extension", robot.pinkArm.getExtensionPosition());
            robot.telemetry.update();
        }

//        robot.shutdown();
    }

}
