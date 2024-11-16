package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.config.Config
import com.outoftheboxrobotics.photoncore.Photon
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.math.Coordinate
import org.firstinspires.ftc.teamcode.robot.init.Robot
import org.firstinspires.ftc.teamcode.robot.init.RobotHardware
import kotlin.math.pow

@Config
@Photon
@TeleOp
class SkibidiTeleop : LinearOpMode() {
    private val hardware = RobotHardware()
    private val robot = Robot()
    private val headingLockAngle = 0.0 // the angle to lock the heading to, in radians
    override fun runOpMode() {
        hardware.init(hardwareMap, LynxModule.BulkCachingMode.MANUAL)
        robot.init(this, hardware, telemetry)
        waitForStart()
        val loopTimer = ElapsedTime()
        val drivingTimer = ElapsedTime()
        val lastGamepad1 = gamepad1
        while (opModeIsActive()) {
            // Manually clear the bulk read cache. Deleting this would be catastrophic b/c stale
            // vals would be used.
            for (hub in hardware.allHubs) {
                hub.clearBulkCache()
            }
            val direction = Coordinate(
                slr(gamepad1.left_stick_x.toDouble()),
                slr(-gamepad1.left_stick_y.toDouble())
            )
            robot.telemetry.addData("x", direction.x)
            robot.telemetry.addData("y", direction.y)
            val rotation = slr(-gamepad1.right_stick_x.toDouble())
            robot.drivetrain.move(
                direction,
                rotation,
                DRIVESPEED
            )

//            if (gamepad1.a && !lastGamepad1.a) { // rising edge
//                robot.inOutTake.intake(new SampleColor[] {
//                        SampleColor.YELLOW,
//                        SampleColor.BLUE
//                });
//            } else if (gamepad1.x && !lastGamepad1.x) { // rising edge
//                robot.inOutTake.outtake();
//            }
//
//            if (gamepad1.y && !lastGamepad1.y) {
//                robot.pinkArm.setExtension(10);
//                robot.pinkArm.setPitch(Math.PI/2);
//            } else if (gamepad1.b && !lastGamepad1.b) {
//                robot.pinkArm.setExtension(100);
//                robot.pinkArm.setPitch(0);
//            }

//            robot.inOutTake.update();
//            robot.pinkArm.update();
            lastGamepad1.copy(gamepad1)
            hardware.write()
            robot.telemetry.addData("Heading Angle", headingLockAngle)
            robot.telemetry.addData("Rotation power", rotation)
            robot.telemetry.addData("Loop time (ms)", loopTimer.milliseconds())
            robot.telemetry.addData("Loop time (hz)", 1000 / loopTimer.milliseconds())
            robot.telemetry.addData("x power", direction.x)
            robot.telemetry.addData("y power", direction.y)
            robot.telemetry.addData("h power", rotation)
            robot.telemetry.update()
            loopTimer.reset()
        }
    }

    fun slr(joystick_value: Double): Double {
        return ((joystick_value.pow(3.0) + joystick_value) / 2).pow(5.0);
    }

    companion object {
        const val DRIVESPEED = 1.0

        //    public static PidfController.PidfCoefficients heading_coeff = new PidfController.PidfCoefficients(0.63, 0, 0.0624, 0, 0.04);
        //
        //    public static PidfController heading = new PidfController(heading_coeff);
        var ROTATION_SPEED = 0.003
    }
}
