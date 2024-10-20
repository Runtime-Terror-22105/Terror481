package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.math.Coordinate.ORIGIN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.drive.localizer.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;

@Config
@Photon
@TeleOp(name = "OTOS tuner", group = "Tuning")
public class TuneOTOS extends LinearOpMode {
    private final RobotHardware hardware = new RobotHardware();
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Drivetrain drivetrain = new MecanumDrivetrain(
                hardware.motorRearLeft,
                hardware.motorFrontLeft,
                hardware.motorRearRight,
                hardware.motorFrontRight
        );

        waitForStart();


        OTOSLocalizer localizer = new OTOSLocalizer(hardware.otos);
        localizer.initializeOtos(new OTOSLocalizer.Parameters(
                // 3.55 inches in the front
                new SparkFunOTOS.Pose2D(0, 3.55, 0),
                new SparkFunOTOS.Pose2D(0, 0, 0),
                1.0,
                1.0
        ));


        // Tuning step 1
        telemetry.addData("Please measure the distance of the OTOS from the center and set it as the offset", "");
        telemetry.addData("Once you have done this, press (a) to continue.", "");
        telemetry.update();
        while (!gamepad1.a) {}


        // Tuning step 2
        double angularScalar = 1;
        while (!gamepad1.a) {
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            localizer.resetTracking();
            telemetry.addData("Spin the robot by 10 rotations, use right joystick", "");
            telemetry.addData("Press (b) once you are complete", "");
            telemetry.update();
            while (!gamepad1.b) {
                for (LynxModule hub : hardware.allHubs) {
                    hub.clearBulkCache();
                }
                drivetrain.move(ORIGIN, -gamepad1.right_stick_y);
                hardware.write();
            }
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            SparkFunOTOS.Pose2D pos = localizer.getPosition();
            double hErr = (10 * 2*Math.PI) - pos.h;
            angularScalar *= 1/hErr;
            localizer.setAngularScalar(angularScalar);

            telemetry.addData("heading error:", hErr);
            telemetry.addData("new angular scalar:", angularScalar);
            telemetry.addData("Press (a) if the error was very little", "");
            telemetry.update();
        }


        // Tuning step 3
        int[] distances = {50, 100, 20};
        double[] linearScalar = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            int distance = distances[i];
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            localizer.resetTracking();
            telemetry.addData("Move the robot " + distance + " inches by hand", "");
            telemetry.addData("Press (b) once you are complete", "");
            telemetry.update();
            while (!gamepad1.b) {}
            for (LynxModule hub : hardware.allHubs) {
                hub.clearBulkCache();
            }

            SparkFunOTOS.Pose2D pos = localizer.getPosition();
            double xErr = distance/pos.x;
            double yErr = distance/pos.y;
            double linearErr = Math.sqrt(xErr*xErr + yErr*yErr);
            linearScalar[i] = 1/linearErr;

            telemetry.addData("y error:", xErr);
            telemetry.addData("x error:", yErr);
            telemetry.addData("linear error:", linearErr);
            telemetry.addData("linear scalar:", linearScalar[i]);
            telemetry.addData("Press (a) if the error was very little", "");
            telemetry.update();
        }
        telemetry.addData("Final linear scalar", avg(linearScalar));

    }

    public double avg(double[] arr) {
        double s = 0;
        for (double n : arr) {
            s += n;
        }
        return s/arr.length;
    }
}
