package org.firstinspires.ftc.teamcode.opmodes.tuning.pinkarm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.init.Robot;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
import org.firstinspires.ftc.teamcode.robot.subsystems.PinkArm;

@Config
@TeleOp(group="Pink Arm Tuning")
public class PitchFFTuner extends LinearOpMode {

    private final RobotHardware hardware = new RobotHardware();
    private final Robot robot = new Robot();

//    private double headingLockAngle = 0; // the angle to lock the heading to, in radians
//    public static PidfController.PidfCoefficients heading_coeff = new PidfController.PidfCoefficients(0.65, 0, 0.065, 0, 0.05);
//    public static PidfController heading = new PidfController(heading_coeff);

    public static double ff_1 = 0.0;

    public static double ff_2 = 0.0;

    public static double extensionDataPoint2 = 0.0;

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
            // User Clicks A to signify Pitch Data Point 1 is done.
            // While loop continues until arm can hold angle
            // Make sure extension is all the way back!
            robot.pinkArm.setExtensionTarget(0); // sets target
            while(!gamepad1.a && opModeIsActive()){
                // bulk read cause its another while loop cause its in another while loop and im too lazy to do states
                for (LynxModule hub : hardware.allHubs) {
                    hub.clearBulkCache();
                }

                // Ensure that extension is good-- we want it to stay at 0
                robot.pinkArm.updateExtension(); // calculates and sets powers to extension motors
                // keep extension constant at 0

                hardware.write();

                robot.telemetry.addData("Current Feed Forward (retracted): ", ff_1);
                double gravityScaled = ff_1*Math.cos(robot.pinkArm.getPitchPosition());
                robot.pinkArm.setPitchPower(gravityScaled); // assigning feedforward power
                robot.telemetry.update();
            }

            // Data point 1 done (with arm all the way back)
            // Sets to max extension
            robot.pinkArm.setExtensionTarget(PinkArm.MAX_EXTENSION);

            while(!gamepad1.a && opModeIsActive()){
                // bulk read cause its another while loop cause its in another while loop and im too lazy to do states
                for (LynxModule hub : hardware.allHubs) {
                    hub.clearBulkCache();
                }

                // PID to keep extension correct
                // It should maintain max extension :)
                robot.pinkArm.updateExtension();

                telemetry.addData("Current Feed Forward (at extension): ", ff_2);
                double gravityScaled = ff_2*Math.cos(robot.pinkArm.getPitchPosition());
                robot.pinkArm.setPitchPower(gravityScaled);
                telemetry.update();
            }

            telemetry.addData("Value 1: ", ff_1);
            telemetry.addData("Value 2: ", ff_2);
            telemetry.update();
            sleep(6000);

        }

//        robot.shutdown();
    }

}
