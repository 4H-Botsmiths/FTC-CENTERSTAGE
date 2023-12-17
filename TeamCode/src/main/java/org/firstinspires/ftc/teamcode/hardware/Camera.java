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
    this.webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
  }

  private WebcamName webcam;
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
  public void initAprilTag() throws CameraNotConnectedException {
    if (!webcam.isAttached()) {
      throw new CameraNotConnectedException();
    }

    // Create the AprilTag processor by using a builder.
    aprilTag = new AprilTagProcessor.Builder().build();
    // Create the vision portal by using a builder.
    visionPortal = new VisionPortal.Builder()
        .setCamera(this.webcam)
        .addProcessor(aprilTag)
        .build();

  } // end method initAprilTag()

  public enum AprilTagPositions {
    LEFT, CENTER, RIGHT
  }

  /**
   * Get the AprilTag detection for the given tag.
   *
   * @param tag The AprilTags enum value representing the tag to detect.
   * @return The AprilTagDetection object representing the detected tag.
   * @throws CameraNotStreamingException If the camera is not streaming.
   * @throws NoTagsFoundException        If no tags are found.
   */
  public AprilTagDetection getAprilTag(AprilTagPositions tag) throws CameraNotStreaming, NoTagsFound, TagNotFound {
    visionPortal.resumeStreaming();
    if (visionPortal.getCameraState() != CameraState.STREAMING) {
      throw new CameraNotStreaming();
    }
    switch (tag) {
      case RIGHT:
        visionPortal.setActiveCamera(webcam);
        break;
      case CENTER:
        visionPortal.setActiveCamera(webcam);
        break;
      case LEFT:
        visionPortal.setActiveCamera(webcam2);
        break;
    }
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      switch (tag) {
        case LEFT:
          if (detection.id == 1 || detection.id == 4) {
            return detection;
          }
          break;
        case CENTER:
          if (detection.id == 2 || detection.id == 5) {
            return detection;
          }
          break;
        case RIGHT:
          if (detection.id == 3 || detection.id == 6) {
            return detection;
          }
          break;
      }
    }
    if (currentDetections.size() == 0) {
      throw new NoTagsFound();
    } else {
      throw new TagNotFound();
    }
  }

  public AprilTagPositions getAvalableAprilTags() throws CameraNotStreaming, NoTagsFound, Exception {
    visionPortal.resumeStreaming();
    if (visionPortal.getCameraState() != CameraState.STREAMING) {
      throw new CameraNotStreaming();
    }
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      if (detection.id == 1 || detection.id == 4) {
        return AprilTagPositions.LEFT;
      }
      if (detection.id == 2 || detection.id == 5) {
        return AprilTagPositions.CENTER;
      }
      if (detection.id == 3 || detection.id == 6) {
        return AprilTagPositions.RIGHT;
      }
    }
    if (currentDetections.size() == 0) {
      throw new NoTagsFound();
    } else {
      throw new Exception("Some unexpected error happened while detecting available tag positions");
    }
  }

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

  /*
   * public enum ActiveCamera {
   * LEFT, RIGHT, NONE
   * }
   * 
   * public void setCamera(ActiveCamera camera) {
   * if (camera == ActiveCamera.LEFT) {
   * visionPortal.resumeStreaming();
   * visionPortal.setActiveCamera(webcam1);
   * } else if (camera == ActiveCamera.RIGHT) {
   * visionPortal.resumeStreaming();
   * visionPortal.setActiveCamera(webcam2);
   * } else if (camera == ActiveCamera.NONE) {
   * visionPortal.stopStreaming();
   * }
   * }
   */
  public class NoTagsFound extends Exception {
    public NoTagsFound() {
      super("No tags were found");
    }
  }

  public class CameraNotStreaming extends Exception {
    public CameraNotStreaming() {
      super("The camera is not streaming");
    }
  }

  public class CameraNotConnected extends Exception {
    public CameraNotConnected() {
      super("The camera is not connected");
    }
  }

  public class AprilTag extends AprilTagDetection {
    public enum AprilTagPosition {
      LEFT, CENTER, RIGHT, UNKNOWN
    }

    public AprilTagPosition position = AprilTagPosition.UNKNOWN;

    public AprilTag(AprilTagDetection detection) {
      //AprilTagDetection​(int id, int hamming, float decisionMargin, org.opencv.core.Point center, org.opencv.core.Point[] corners, AprilTagMetadata metadata, AprilTagPoseFtc ftcPose, AprilTagPoseRaw rawPose, long frameAcquisitionNanoTime)
      super(detection.id, detection.hamming, detection.decisionMargin, detection.center, detection.corners,
          detection.metadata, detection.ftcPose, detection.rawPose, detection.frameAcquisitionNanoTime);
      if (detection.id == 1 || detection.id == 4) {
        position = AprilTagPosition.LEFT;
      }
      if (detection.id == 2 || detection.id == 5) {
        position = AprilTagPosition.CENTER;
      }
      if (detection.id == 3 || detection.id == 6) {
        position = AprilTagPosition.RIGHT;
      }
    }
  }
} // end class
