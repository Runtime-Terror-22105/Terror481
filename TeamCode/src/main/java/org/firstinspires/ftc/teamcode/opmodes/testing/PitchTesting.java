package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Autonomous(group = "testing")
public class PitchTesting extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        hardware.armPitchMotor1.setPower(0);
        hardware.armPitchMotor2.setPower(0);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < 200) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            // region Driving
            hardware.armPitchMotor1.setPower(1.0);
            hardware.armPitchMotor2.setPower(1.0);
//            hardware.armPitchMotor2.setPower(-1.0);
            // endregion driving
            hardware.write();
        }

        hardware.armPitchMotor1.setPower(0);
        hardware.armPitchMotor2.setPower(0);
        hardware.write();
    }

}