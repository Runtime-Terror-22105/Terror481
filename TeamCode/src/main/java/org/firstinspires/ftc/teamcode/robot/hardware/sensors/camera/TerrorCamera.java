package org.firstinspires.ftc.teamcode.robot.hardware.sensors.camera;

import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class TerrorCamera {
    public static final float TAG_DETECTION_THRESHOLD = 0.2f; // TODO: tune this value

    // Camera stuff
    @NonNull public VisionPortal visionPortal;
    @Nullable public AprilTagProcessor tagProcessor;

    private TerrorCamera(@NonNull VisionPortal visionPortal, @Nullable AprilTagProcessor tagProcessor) {
        this.visionPortal = visionPortal;
        this.tagProcessor = tagProcessor;
    }

    public static class VisionPortalInitialization {
        private final VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        private AprilTagProcessor tagProcessor = null;

        public VisionPortalInitialization setCameraResolution(@NonNull Size cameraResolution) {
            this.visionPortalBuilder.setCameraResolution(cameraResolution);
            return this;
        }

        public VisionPortalInitialization setCamera(@NonNull CameraName camera) {
            this.visionPortalBuilder.setCamera(camera);
            return this;
        }

        public VisionPortalInitialization detectAprilTags() {
            this.tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setSuppressCalibrationWarnings(false)
//                    .setLensIntrinsics() // TODO: placeholder to remind us to calibrate the camera
//                    .setTagFamily() // TODO: placeholder
//                    .setTagLibrary() // TODO: placeholder
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS) // TODO: Placeholder
                    .setNumThreads(3) // TODO: the default is 3 but maybe we can change
                    .build();

            this.visionPortalBuilder.addProcessor(this.tagProcessor);
            return this;
        }

//    public Builder useEOCVPipeline() {
//        TerrorProcessor imgProcessor = new TerrorProcessor(dashboard, telemetry);
//        this.visionPortalBuilder.addProcessor(imgProcessor);
//        return this;
//    }

        public TerrorCamera init() {
            return new TerrorCamera(this.visionPortalBuilder.build(), this.tagProcessor);
        }
    }
}
