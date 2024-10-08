package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.teamcode.cvpipelines.processors.PipelineThatSharesInformationAsVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

public class CVSubsystem {

    private final Size CAMERA_RESOLUTION = new Size(640, 480);

    private PipelineThatSharesInformationAsVisionProcessor processor1;


    public CVSubsystem(WebcamName cameraName, LinearOpMode opMode) {
        VisionPortal visionPortal = buildVisionPortal(cameraName);

        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING);

        // we update exposure just to reduce motion blur (for our cameras, decreasing exposure decreases shutter speed)
        updateExposure(visionPortal);
    }

    public VisionPortal buildVisionPortal(WebcamName cameraName) {
        processor1 = new PipelineThatSharesInformationAsVisionProcessor();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(cameraName)
                .setCameraResolution(CAMERA_RESOLUTION)
                .setAutoStopLiveView(false)
                .addProcessors(processor1)  // ADD PROCESSORS HERE
                .build();

        visionPortal.setProcessorEnabled(processor1, true);  // let processors run asynchronously using camera data

        return visionPortal;
    }

    public void updateExposure(VisionPortal visionPortal) {
        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(15, TimeUnit.MILLISECONDS);  // exposure may have to be adjusted during competitions
    }

    public int getDetectedOrangePixels() {
        // method value will remain up to date after each frame since processor1 is added and active in the vision portal
        return processor1.getDetectedPixels();
    }

}
