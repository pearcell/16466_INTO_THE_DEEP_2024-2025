package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests.Auto_Test;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import java.util.List;

public class Vision {

    ColorBlobLocatorProcessor colorLocatorYellow = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(24, 50, 100), new Scalar(54, 255, 255)))         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asImageCoordinates(0, 0, 320, 240))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5)                               // Smooth the transitions between different colors in image
            .build();

    ColorBlobLocatorProcessor colorLocatorBlue = new ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
            .setRoi(ImageRegion.asImageCoordinates(0, 0, 320, 240))  // search central 1/4 of camera view
            .setDrawContours(true)                        // Show contours on the Stream Preview
            .setBlurSize(5) // Smooth the transitions between different colors in image
            .build();

    ColorBlobLocatorProcessor.BlobFilter areaFilter = new ColorBlobLocatorProcessor.BlobFilter(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 1000, 10000);

    public void addFilters() {
        colorLocatorBlue.addFilter(areaFilter);
        colorLocatorYellow.addFilter(areaFilter);
    }

    MecanumDrive drive;

    public Vision(HardwareMap hardwareMap, MecanumDrive drive1) {

        drive = drive1;

        addFilters();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocatorYellow)
                .addProcessor(colorLocatorBlue)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public boolean Select(ColorBlobLocatorProcessor processor1, ColorBlobLocatorProcessor processor2) {

        List<ColorBlobLocatorProcessor.Blob> blobs1 = processor1.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> blobs2 = processor2.getBlobs();

        if (blobs2 != null) {
            blobs1.addAll(blobs2);
        }

        double angle;
        boolean flag = false;

        for(ColorBlobLocatorProcessor.Blob b : blobs1)
        {
            RotatedRect boxFit = b.getBoxFit();
            angle = boxFit.angle;

            if (boxFit.size.width > boxFit.size.height) { // ensure consistent angle measurements from 0 to 180
                angle = angle + 90;
            }

            double servoPos = ((-0.7 / 180) * angle) + .85; //convert angles to servo position

            if (boxFit.center.x > 130 && boxFit.center.x < 190 && boxFit.center.y > 90 && boxFit.center.y < 150) {
                flag = true;
                Auto_Test.angleCVTest = servoPos;
            }
        }

        if (flag) {
            return false;
        } else {
            return true;
        }
    }

    public class Scan implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            return Select(colorLocatorYellow, colorLocatorBlue); // boolean value which returns false when a desired sample is found
        }


    }

    public Action scan() {
        return new Scan();
    }

    public class UpdatePose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            drive.updatePoseEstimate();

            TrajectoryActionBuilder reOrient = drive.actionBuilder(drive.pose)
                    .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y + 2.5), new TranslationalVelConstraint(3));

            Actions.runBlocking(
                    reOrient.build()
            );

            return false;
        }
    }

    public Action updatePose() {
        return new UpdatePose();
    }

}
