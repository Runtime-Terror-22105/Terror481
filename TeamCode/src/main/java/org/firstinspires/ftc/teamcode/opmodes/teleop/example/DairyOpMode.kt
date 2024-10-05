package org.firstinspires.ftc.teamcode.opmodes.teleop.example

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import dev.frozenmilk.dairy.core.FeatureRegistrar

// add feature annotations here
class DairyOpMode : LinearOpMode() {
    private fun _init() {

    }

    override fun runOpMode() {
        // DO NOT put code before this
        FeatureRegistrar.checkFeatures(/* pass desired features as varargs here */)
        FeatureRegistrar.opModePreInit(FeatureRegistrar.activeOpModeWrapper)

        this._init()

        // remember that you can use OpModeLazyCells to init your hardware and similar
        FeatureRegistrar.opModePostInit(FeatureRegistrar.activeOpModeWrapper)
        while (opModeInInit()) {
            FeatureRegistrar.opModePreInitLoop(FeatureRegistrar.activeOpModeWrapper)
            // your init_loop code here
            FeatureRegistrar.opModePostInitLoop(FeatureRegistrar.activeOpModeWrapper)
        }

        waitForStart()

        FeatureRegistrar.opModePreStart(FeatureRegistrar.activeOpModeWrapper)
        // your start code here
        FeatureRegistrar.opModePostStart(FeatureRegistrar.activeOpModeWrapper)

        while (opModeIsActive()) {
            FeatureRegistrar.opModePreLoop(FeatureRegistrar.activeOpModeWrapper)
            this.loop()
            FeatureRegistrar.opModePostLoop(FeatureRegistrar.activeOpModeWrapper)
        }
        FeatureRegistrar.opModePreStop(FeatureRegistrar.activeOpModeWrapper)
        // your stop code here
        FeatureRegistrar.opModePostStop(FeatureRegistrar.activeOpModeWrapper)
    }
}