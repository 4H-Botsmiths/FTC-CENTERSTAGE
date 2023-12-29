package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Comparator;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.classes.DCMotor;
import org.firstinspires.ftc.teamcode.classes.HDMotor;
import org.firstinspires.ftc.teamcode.hardware.Camera;
import org.firstinspires.ftc.teamcode.classes.CoreMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop
 * period of an FTC match.
 * The names of OpModes appear on the menu of the FTC robot.Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class Basic instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank robot.Drive Teleop for a two
 * wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the robot.Driver
 * Station OpMode list
 */

@TeleOp(name = "Camera Teleop", group = "B")
// @Disabled
public class CameraTeleop extends OpMode {
  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private Robot robot = null;
  private Camera camera = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    // Initialize the hardware variables. Note that the strings used here as
    // parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    robot = new Robot(hardwareMap);
    camera = new Camera(hardwareMap, telemetry);
    try {
      camera.initAprilTag();
    } catch (Camera.CameraNotAttachedException e) {
      //TODO: How should this be handled?
    }

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    /*
     * rearRight.setDirection(DcMotor.Direction.REVERSE);
     * rearLeft.setDirection(DcMotor.Direction.REVERSE);
     * frontRight.setDirection(DcMotor.Direction.FORWARD);
     * frontLeft.setDirection(DcMotor.Direction.FORWARD);
     */

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    camera.telemetryAprilTag();
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    //camera.visionPortal.stopLiveView();
    try {
      camera.pause();
    } catch (Camera.CameraNotAttachedException e) {
      //Do Nothing
    }
    runtime.reset();
  }

  public void driverLoop() {
    double x = gamepad1.left_stick_x / 3;
    x += gamepad1.right_bumper ? gamepad1.left_stick_x / 3 : 0;
    x += gamepad1.left_bumper ? gamepad1.left_stick_x / 3 : 0;
    double y = -gamepad1.left_stick_y / 3;
    y += gamepad1.right_bumper ? -gamepad1.left_stick_y / 3 : 0;
    y += gamepad1.left_bumper ? -gamepad1.left_stick_y / 3 : 0;
    double z = gamepad1.right_stick_x / 3;
    z += gamepad1.right_bumper ? gamepad1.right_stick_x / 3 : 0;
    z += gamepad1.left_bumper ? gamepad1.right_stick_x / 3 : 0;

    // robot.Drive System
    robot.Drive(x, y, z);
  }

  double elbowPosition = 0;
  double trapdoorPosition = 0;

  public void operatorLoop() {

    /*
     * NOTES:
     * what needs to be controlled:
     * > Intake - left trigger DONE
     * > Trapdoor - a DONE
     * > Lift - left joystick DONE
     * > Elbow - right joystick
     */
    if (gamepad2.a) {
      robot.trapdoor.setPosition(0.4);
      robot.intake.setSpeed(0.2);
    } else {
      if (gamepad2.right_bumper) {
        robot.intake.setSpeed(1);
        robot.trapdoor.setPosition(0.75);
      } else if (gamepad2.left_bumper) {
        robot.intake.setSpeed(-0.25);
        robot.trapdoor.setPosition(1);
      } else {
        robot.intake.setSpeed(0);
        robot.trapdoor.setPosition(1);
      }
      //robot.intake.setSpeed(gamepad2.right_bumper ? 1 : gamepad2.left_bumper ? -0.5 : 0);
    }
    if (gamepad2.dpad_up) {
      robot.lift.expand();
    } else if (gamepad2.dpad_down) {
      robot.lift.constrict();
    }
    robot.lift.setSpeed(-gamepad2.left_stick_y * 0.5);

    /** 0 = down; 1 = up */
    robot.lift.setPosition(elbowPosition);
    elbowPosition += -gamepad2.right_stick_y * 0.01;
    elbowPosition = elbowPosition > 1 ? 1 : elbowPosition < 0 ? 0 : elbowPosition;
    //Same as above except it should control the trapdoor based off of the right_stick_x
    /*robot.trapdoor.setPosition(trapdoorPosition);
    trapdoorPosition += gamepad2.right_stick_x * 0.01;
    trapdoorPosition = trapdoorPosition > 1 ? 1 : trapdoorPosition < 0 ? 0 : trapdoorPosition;*/
    /*
     * final double lowerLimit = 0.03;
     * final double upperLimit = 0.4;
     * elbowPosition = elbowPosition > upperLimit ? upperLimit
     * : elbowPosition < lowerLimit ? lowerLimit : elbowPosition;
     */

    /*
     * robot.leftElbow.setPosition(-gamepad2.right_stick_y);
     * robot.rightElbow.setPosition(1 + gamepad2.right_stick_y);
     */
  }

  public void superuserLoop(Gamepad gamepad) {
    double x = gamepad.left_stick_x / 3;
    double y = -gamepad.left_stick_y / 3;
    double z = gamepad.right_stick_x / 3;

    // robot.Drive System
    robot.Drive(x, y, z);

    /*
     * NOTES:
     * what needs to be controlled:
     * > Intake - left trigger DONE
     * > Trapdoor - a DONE
     * > Lift - left joystick DONE
     * > Elbow - right joystick
     */
    if (gamepad2.a) {
      robot.trapdoor.setPosition(0.4);
      robot.intake.setSpeed(0.2);
    } else {
      if (gamepad2.right_bumper) {
        robot.intake.setSpeed(1);
        robot.trapdoor.setPosition(0.75);
      } else if (gamepad2.left_bumper) {
        robot.intake.setSpeed(-0.25);
        robot.trapdoor.setPosition(1);
      } else {
        robot.intake.setSpeed(0);
        robot.trapdoor.setPosition(1);
      }
      //robot.intake.setSpeed(gamepad2.right_bumper ? 1 : gamepad2.left_bumper ? -0.5 : 0);
    }
    if (gamepad.dpad_up) {
      robot.lift.expand();
    } else if (gamepad.dpad_down) {
      robot.lift.constrict();
    }
    robot.lift.setSpeed(gamepad.dpad_left ? -0.5 : gamepad.dpad_right ? 0.5 : 0);

    /** 0 = down; 1 = up */
    robot.lift.setPosition(elbowPosition);
    elbowPosition += -gamepad2.right_stick_y * 0.01;
    elbowPosition = elbowPosition > 1 ? 1 : elbowPosition < 0 ? 0 : elbowPosition;
    /*
     * final double lowerLimit = 0.03;
     * final double upperLimit = 0.4;
     * elbowPosition = elbowPosition > upperLimit ? upperLimit
     * : elbowPosition < lowerLimit ? lowerLimit : elbowPosition;
     */

    /*
     * robot.leftElbow.setPosition(-gamepad2.right_stick_y);
     * robot.rightElbow.setPosition(1 + gamepad2.right_stick_y);
     */
  }

  /*
   * left: x, bottom: a, right: b
   * if one user hit x, vibrate the left side of the other controller
   * if one user hit a, vibrate both sides of the other controller
   * if one user hit b, vibrate the right side of the other controller
   * 
   * if both users are hitting the same button, make phone say 'Placing Pixel',
   * place the pixel at the
   * corresponding location
   * 
   * if X, use the right camera and align it to the center id
   * if B, use the left camera and align it to the center id
   * if A, use either camera and align it to the center id
   * 
   * once aligned, raise lift (use encoder to know when to stop)
   * once stopped, lift elbow (servos will know when to stop)
   * once elbow is raised, drive forward using cameras to know when to stop
   * open trapdoor and slowly spin intake backwards. stop after certain period of
   * time (or use color sensors placed in intake to watch for pixels?)
   * make phone say 'Pixel Placed'
   * 
   * DONE
   */
  enum PixelPosition {
    NONE, LEFT, CENTER, RIGHT, UNDECIDED
  }

  public PixelPosition getPosition() {
    PixelPosition driverChoice = PixelPosition.NONE;
    if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5) {
      driverChoice = PixelPosition.CENTER;
    } else if (gamepad1.left_trigger > 0.5) {
      driverChoice = PixelPosition.LEFT;
    } else if (gamepad1.right_trigger > 0.5) {
      driverChoice = PixelPosition.RIGHT;
    }
    /*if (gamepad1.x) {
      driverChoice = PixelPosition.LEFT;
    } else if (gamepad1.a) {
      driverChoice = PixelPosition.CENTER;
    } else if (gamepad1.b) {
      driverChoice = PixelPosition.RIGHT;
    }*/
    PixelPosition operatorChoice = PixelPosition.NONE;
    if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5) {
      operatorChoice = PixelPosition.CENTER;
    } else if (gamepad2.left_trigger > 0.5) {
      operatorChoice = PixelPosition.LEFT;
    } else if (gamepad2.right_trigger > 0.5) {
      operatorChoice = PixelPosition.RIGHT;
    }
    /*if (gamepad2.x) {
      operatorChoice = PixelPosition.LEFT;
    } else if (gamepad2.a) {
      operatorChoice = PixelPosition.CENTER;
    } else if (gamepad2.b) {
      operatorChoice = PixelPosition.RIGHT;
    }*/
    if (driverChoice == operatorChoice) {
      if (driverChoice == PixelPosition.NONE) {
        gamepad1.stopRumble();
        gamepad2.stopRumble();
      }
      return driverChoice;
    } else {
      switch (driverChoice) {
        case NONE:
          gamepad2.stopRumble();
          break;
        case LEFT:
          gamepad2.rumble(0.5, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case CENTER:
          gamepad2.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case RIGHT:
          gamepad2.rumble(0, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
      }
      switch (operatorChoice) {
        case NONE:
          gamepad1.stopRumble();
          break;
        case LEFT:
          gamepad1.rumble(0.5, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case CENTER:
          gamepad1.rumble(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case RIGHT:
          gamepad1.rumble(0, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
      }
      return PixelPosition.UNDECIDED;
    }
  }

  public PixelPosition getPosition(Gamepad gamepad) {
    PixelPosition superuserChoice = PixelPosition.NONE;
    if (gamepad.left_trigger > 0.5 && gamepad.right_trigger > 0.5) {
      superuserChoice = PixelPosition.CENTER;
    } else if (gamepad.left_trigger > 0.5) {
      superuserChoice = PixelPosition.LEFT;
    } else if (gamepad.right_trigger > 0.5) {
      superuserChoice = PixelPosition.RIGHT;
    }
    if (superuserChoice == PixelPosition.NONE) {
      gamepad.stopRumble();
    }
    return superuserChoice;
  }

  public void reset() {
    robot.Drive(0, 0, 0);
    robot.lift.constrict();
    robot.intake.setSpeed(0);
    robot.trapdoor.setPosition(0);
    try {
      camera.pause();
    } catch (Camera.CameraNotAttachedException e) {
      //Do Nothing
    }
  }

  // degrees are more sensitive than inches, so they need finer control
  public final double turnSensitivity = 0.05;
  public final double sensitivity = 0.1;
  public final double speedLimit = 0.15;
  public final double turnSpeedLimit = 0.1;
  public final double precisionSpeedLimit = 0.15;

  public final double pressureSpeed = 0.1;

  public final double tolerance = 5;
  public final double precisionTolerance = 1.5;

  /**How far the front of the robot should be from the board */
  public final double DISTANCE = 10; //6 final distance?
  public final double FINAL_DISTANCE = 6;

  public boolean spokeAligning = false;
  public boolean spokeApproaching = false;
  public boolean spokePlacing = false;

  public void placePixel(Camera.AprilTagPosition position) {
    try {
      camera.resume();
      robot.lift.expand();
      Gamepad gamepad = superuser == null ? gamepad1 : superuser;
      try {
        List<Camera.AprilTag> tags = camera.getAprilTags();

        Camera.AprilTag tag = null;
        for (Camera.AprilTag _tag : tags) {
          if (_tag.position == position) {
            tag = _tag;
          }
        }
        if (tag != null) {
          /* Old algorithm:
          robot.Drive(currentDetection.ftcPose.x > 1 ? 0.1 : currentDetection.ftcPose.x < -1 ? -0.1 : 0,
                            currentDetection.ftcPose.range > 20 ? 0.1 : currentDetection.ftcPose.range < 18 ? -0.1 : 0,
                            currentDetection.ftcPose.yaw > 1 ? -0.1
                                    : currentDetection.ftcPose.yaw < -1 ? 0.1 : 0);
          */
          boolean aligned = tag.ftcPose.x < tolerance && tag.ftcPose.x > -tolerance
              && tag.ftcPose.range < DISTANCE + tolerance
              && tag.ftcPose.yaw < tolerance && tag.ftcPose.yaw > -tolerance;
          boolean ready = tag.ftcPose.x < precisionTolerance && tag.ftcPose.x > -precisionTolerance
              && tag.ftcPose.range < FINAL_DISTANCE + precisionTolerance
              && tag.ftcPose.yaw < precisionTolerance && tag.ftcPose.yaw > -precisionTolerance;
          double forwardSpeed = Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit);
          if (ready) {
            robot.trapdoor.setPosition(0.4);
            robot.intake.setSpeed(0.5);
            forwardSpeed = Range.clip(((tag.ftcPose.range - FINAL_DISTANCE) * sensitivity) + pressureSpeed,
                -precisionSpeedLimit,
                precisionSpeedLimit);
            telemetry.addLine("Placing");
            if (!spokePlacing) {
              telemetry.speak("Placing");
              spokePlacing = true;
            }
            spokeAligning = false;
            spokeApproaching = false;
          } else if (aligned) {
            forwardSpeed = Range.clip((tag.ftcPose.range - FINAL_DISTANCE) * sensitivity, -precisionSpeedLimit,
                precisionSpeedLimit);
            telemetry.addLine("Approaching");
            if (!spokeApproaching) {
              telemetry.speak("Approaching");
              spokeApproaching = true;
            }
            spokeAligning = false;
            spokePlacing = false;
          } else {
            telemetry.addLine("Aligning");
            if (!spokeAligning) {
              telemetry.speak("Aligning");
              spokeAligning = true;
            }
            spokePlacing = false;
            spokeApproaching = false;
          }
          robot.Drive(Range.clip(tag.ftcPose.x * sensitivity, -speedLimit, speedLimit) + (gamepad.left_stick_x / 3),
              forwardSpeed + (-gamepad.left_stick_y / 3),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit)
                  + (gamepad.right_stick_x / 3));
        } else {
          for (Camera.AprilTag _tag : tags) {
            if (_tag.position != Camera.AprilTagPosition.CENTER) {
              tag = _tag;
            }
          }
          tag = tag == null ? tags.get(0) : tag;
          switch (position) {
            case LEFT:
              robot.Drive(
                  (tag.position == Camera.AprilTagPosition.CENTER ? -0.2 : -0.3) + (gamepad.left_stick_x / 3),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit)
                      + (-gamepad.left_stick_y / 3),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit)
                      + (gamepad.right_stick_x / 3));
              break;
            case RIGHT:
              robot.Drive((tag.position == Camera.AprilTagPosition.CENTER ? 0.2 : 0.3) + (gamepad.left_stick_x / 3),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit)
                      + (-gamepad.left_stick_y / 3),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit)
                      + (gamepad.right_stick_x / 3));
              break;
            case CENTER:
              robot.Drive((tag.position == Camera.AprilTagPosition.LEFT ? 0.2 : -0.2) + (gamepad.left_stick_x / 3),
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit)
                      + (-gamepad.left_stick_y / 3),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit)
                      + (gamepad.right_stick_x / 3));
              break;
          }
        }
        gamepad1.stopRumble();
        gamepad2.stopRumble();
      } catch (Camera.CameraNotStreamingException e) {
        robot.Drive(gamepad.left_stick_x / 3, -gamepad.left_stick_y / 3, gamepad.right_stick_x / 3);
        if (superuser != null) {
          superuser.rumble(0.25, 0.25, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
          gamepad1.rumble(0.25, 0.25, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          gamepad2.rumble(0.25, 0.25, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        }
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        if (gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0 || gamepad.right_stick_x != 0) {
          robot.Drive(gamepad.left_stick_x / 3, -gamepad.left_stick_y / 3, gamepad.right_stick_x / 3);
        } else {
          robot.Drive(0.25); //Reduce speed by 25%
        }
        if (superuser != null) {
          superuser.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else {
          gamepad1.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          gamepad2.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        }
      }
    } catch (Camera.CameraNotAttachedException e) {
      //Function can't run
      telemetry.speak("Camera is not attached");
      //TODO: notify user
    }
  }

  PixelPosition lastPosition = PixelPosition.NONE;
  Gamepad superuser = null;

  /*
  * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
  */
  @Override
  public void loop() {
    telemetry.addData("Status", superuser == null ? "Running" : "Running - Superuser Active");
    telemetry.addData("Voltage", Math.round(robot.voltage.getVoltage() * 100) / 100);
    telemetry.addData("Average Voltage", Math.round(robot.averageVoltage * 100) / 100);
    telemetry.addData("Performance", "%d%%", Math.round(robot.performance * 100));
    telemetry.addData("Front Left", Math.round(robot.frontLeft.getSpeed() * 100));
    telemetry.addData("Front Right", Math.round(robot.frontRight.getSpeed() * 100));
    telemetry.addData("Rear Right", Math.round(robot.rearRight.getSpeed() * 100));
    telemetry.addData("Rear Left", Math.round(robot.rearLeft.getSpeed() * 100));

    telemetry.addData("Intake", Math.round(robot.rearLeft.getSpeed() * 100));
    telemetry.addData("Trapdoor", Math.round(robot.trapdoor.getPosition() * 100));
    robot.lift.telemetries(telemetry);
    telemetry.addData("Elbow Command", Math.round(elbowPosition * 100));

    if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
      superuser = gamepad1;
    } else if (gamepad2.a && gamepad2.b && gamepad2.x && gamepad2.y) {
      superuser = gamepad2;
    } else if (gamepad1.b && gamepad2.b) {
      superuser = null;
    }
    handleLoop();
    camera.telemetryAprilTag();
  }

  public void handleLoop() {

    PixelPosition position = superuser != null ? getPosition(superuser) : getPosition();
    if (position == PixelPosition.NONE) {
      if (lastPosition != PixelPosition.NONE) {
        reset();
      } else {
        if (superuser != null) {
          superuserLoop(superuser);
        } else {
          driverLoop();
          operatorLoop();
        }
      }
    } else if (position == PixelPosition.UNDECIDED) {
      if (superuser != null) {
        superuserLoop(superuser);
      } else {
        driverLoop();
        operatorLoop();
      }
    } else {
      switch (position) {
        case LEFT:
          placePixel(Camera.AprilTagPosition.LEFT);
          break;
        case CENTER:
          placePixel(Camera.AprilTagPosition.CENTER);
          break;
        case RIGHT:
          placePixel(Camera.AprilTagPosition.RIGHT);
          break;
        case NONE:
          //Will never happen due to if/else loop
          //This statement is included for intellisense warnings
          break;
      }
      //lastTask = placePixel(position);
    }
    lastPosition = position;
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
  }
}
