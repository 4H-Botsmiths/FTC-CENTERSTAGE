package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation, using
 * two webcams.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
public class Camera {

  /*
   * Variables used for switching cameras.
   */
  public Camera(HardwareMap hardwareMap) {
    this.webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
    this.webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
  }

  private WebcamName webcam1, webcam2;
  /**
   * The variable to store our instance of the AprilTag processor.
   */
  private AprilTagProcessor aprilTag;

  /**
   * The variable to store our instance of the vision portal.
   */
  public VisionPortal visionPortal;

  /**
   * Initialize the AprilTag processor.
   */
  public void initAprilTag() {

    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder().build();

    CameraName switchableCamera = ClassFactory.getInstance()
        .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

    // Create the vision portal by using a builder.
    visionPortal = new VisionPortal.Builder()
        .setCamera(switchableCamera)
        .addProcessor(aprilTag)
        .build();

  } // end method initAprilTag()

  /**
   * Add telemetry about AprilTag detections.
   */
  public void telemetryAprilTag(Telemetry telemetry) {

    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    telemetry.addData("# AprilTags Detected", currentDetections.size());

    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      if (detection.metadata != null) {
        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y,
            detection.ftcPose.z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll,
            detection.ftcPose.yaw));
        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range,
            detection.ftcPose.bearing, detection.ftcPose.elevation));
      } else {
        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
      }
    } // end for() loop

    // Add "key" information to telemetry
    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");

  } // end method telemetryAprilTag()

  public enum ActiveCamera {
    LEFT, RIGHT, NONE
  }

  public void setCamera(ActiveCamera camera) {
    if (camera == ActiveCamera.LEFT) {
      visionPortal.resumeStreaming();
      visionPortal.setActiveCamera(webcam1);
    } else if (camera == ActiveCamera.RIGHT) {
      visionPortal.resumeStreaming();
      visionPortal.setActiveCamera(webcam2);
    } else if (camera == ActiveCamera.NONE) {
      visionPortal.stopStreaming();
    }
  }

} // end class
