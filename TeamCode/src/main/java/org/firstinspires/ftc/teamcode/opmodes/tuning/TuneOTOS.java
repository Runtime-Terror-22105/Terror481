//package org.firstinspires.ftc.teamcode.opmodes.tuning;
//
//import static org.firstinspires.ftc.teamcode.math.Coordinate.ORIGIN;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.math.Pose2d;
//import org.firstinspires.ftc.teamcode.robot.drive.Drivetrain;
//import org.firstinspires.ftc.teamcode.robot.drive.localizer.OTOSLocalizer;
//import org.firstinspires.ftc.teamcode.robot.drive.mecanum.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.robot.init.RobotHardware;
//
//@Config
//@TeleOp(name = "OTOS tuner", group = "Tuning")
//public class TuneOTOS extends LinearOpMode {
//    private final RobotHardware hardware = new RobotHardware();
//    private FtcDashboard dashboard;
//
//    @Override
//    public void runOpMode() {
//        hardware.init(hardwareMap, LynxModule.BulkCachingMode.AUTO);
//
//        dashboard = FtcDashboard.getInstance();
//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
//        Drivetrain drivetrain = new MecanumDrivetrain(
//                hardware.motorRearLeft,
//                hardware.motorFrontLeft,
//                hardware.motorRearRight,
//                hardware.motorFrontRight
//        );
//
//        waitForStart();
//
//
//        OTOSLocalizer localizer = new OTOSLocalizer(hardware.otos);
//        localizer.initializeOtos(new OTOSLocalizer.Parameters(
//                // 3.55 inches in the front
//                new Pose2d(0, 3.55, 0),
//                new Pose2d(0, 0, 0),
//                1.0,
//                1.0
//        ));
//
//
//        // Tuning step 1
//        telemetry.addData("Please measure the distance of the OTOS from the center and set it as the offset", "");
//        telemetry.addData("Once you have done this, press (a) to continue.", "");
//        telemetry.update();
//        while (!gamepad1.a && opModeIsActive()) {}
//
//        sleep(1000);
//
//
//        // Tuning step 2
//        localizer.resetTracking();
//        telemetry.addData("Spin the robot by 10 rotations, use right joystick", "");
//        telemetry.addData("Press (b) once you are complete", "");
//        telemetry.update();
//        while (!gamepad1.b && opModeIsActive()) {
//            drivetrain.move(ORIGIN, -gamepad1.right_stick_x);
//            hardware.write();
//        }
//
//        double headingErr = Math.toDegrees(localizer.getPosition().heading);
//        double angularScalar = 3600/(3600+headingErr);
//        localizer.setAngularScalar(angularScalar);
//
//        telemetry.addData("heading in degrees:", headingErr);
//        telemetry.addData("new angular scalar:", angularScalar);
//        telemetry.update();
//        sleep(5000);
//
//        // Tuning step 3
//        int[] distances = {24, 96, 48};
//        double[] linearScalar = {0, 0, 0};
//        for (int i = 0; i < 3; i++) {
//            int distance = distances[i];
//
//            localizer.resetTracking();
//            telemetry.addData("Move the robot " + distance + " inches by hand", "");
//            telemetry.addData("Press (b) once you are complete", "");
//            telemetry.update();
//            while (!gamepad1.b && opModeIsActive()) {}
//
//            Pose2d pos = localizer.getPosition();
//            double xErr = distance/pos.x;
//            double yErr = distance/pos.y;
//            double linearDist = Math.sqrt(pos.x*pos.x + pos.y*pos.y);
//            linearScalar[i] = distance/linearDist;
//
//            telemetry.addData("y error:", xErr);
//            telemetry.addData("x error:", yErr);
//            telemetry.addData("linear error:", linearDist);
//            telemetry.addData("linear scalar:", linearScalar[i]);
//            telemetry.update();
//        }
//        telemetry.addData("Final linear scalar", avg(linearScalar));
//
//    }
//
//    public double avg(@NonNull double[] arr) {
//        double s = 0;
//        for (double n : arr) {
//            s += n;
//        }
//        return s/arr.length;
//    }
//}
