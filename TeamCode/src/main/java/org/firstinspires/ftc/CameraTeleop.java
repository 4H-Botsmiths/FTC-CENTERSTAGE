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
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class Basic instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two
 * wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code
 * folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver
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
    double x = 0.625 * gamepad1.left_stick_x + 0.375 * gamepad1.left_stick_x * gamepad1.right_trigger
        - 0.375 * gamepad1.left_stick_x * gamepad1.left_trigger;
    double y = 0.625 * -gamepad1.left_stick_y + 0.375 * -gamepad1.left_stick_y * gamepad1.right_trigger
        - 0.375 * -gamepad1.left_stick_y * gamepad1.left_trigger;
    double z = 0.625 * gamepad1.right_stick_x + 0.375 * gamepad1.right_stick_x * gamepad1.right_trigger
        - 0.375 * gamepad1.right_stick_x * gamepad1.left_trigger;

    // Drive System
    Drive(x, y, z);
  }

  double elbowPosition = 0;

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
      robot.trapdoor.setPosition(0);
      robot.intake.setSpeed(0.2);
    } else {
      robot.trapdoor.setPosition(1);

      robot.intake.setSpeed(gamepad2.right_trigger > 0 ? -gamepad2.right_trigger : gamepad2.left_trigger);
    }
    if (gamepad2.dpad_up) {
      robot.lift.raise();
    } else if (gamepad2.dpad_down) {
      robot.lift.lower();
    }
    robot.lift.setSpeed(-gamepad2.left_stick_y * 0.5);

    /** 0 = down; 1 = up */
    robot.leftElbow.setPosition(elbowPosition);
    robot.rightElbow.setPosition(elbowPosition);
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

  public void superuserLoop(Gamepad gamepad) {
    double x = 0.625 * gamepad.left_stick_x + 0.375 * gamepad.left_stick_x * gamepad.right_trigger
        - 0.375 * gamepad.left_stick_x * gamepad.left_trigger;
    double y = 0.625 * -gamepad.left_stick_y + 0.375 * -gamepad.left_stick_y * gamepad.right_trigger
        - 0.375 * -gamepad.left_stick_y * gamepad.left_trigger;
    double z = 0.625 * gamepad.right_stick_x + 0.375 * gamepad.right_stick_x * gamepad.right_trigger
        - 0.375 * gamepad.right_stick_x * gamepad.left_trigger;

    // Drive System
    Drive(x, y, z);

    /*
     * NOTES:
     * what needs to be controlled:
     * > Intake - left trigger DONE
     * > Trapdoor - a DONE
     * > Lift - left joystick DONE
     * > Elbow - right joystick
     */
    if (gamepad.a) {
      robot.trapdoor.setPosition(0);
      robot.intake.setSpeed(0.2);
    } else {
      robot.trapdoor.setPosition(1);

      robot.intake.setSpeed(gamepad2.right_trigger > 0 ? -gamepad2.right_trigger : gamepad2.left_trigger);
    }
    if (gamepad.dpad_up) {
      robot.lift.raise();
    } else if (gamepad.dpad_down) {
      robot.lift.lower();
    }
    robot.lift.setSpeed(gamepad.dpad_left ? -0.5 : gamepad.dpad_right ? 0.5 : 0);

    /** 0 = down; 1 = up */
    robot.leftElbow.setPosition(elbowPosition);
    robot.rightElbow.setPosition(elbowPosition);
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
    NONE, LEFT, CENTER, RIGHT
  }

  public PixelPosition getPosition() {
    PixelPosition driverChoice = PixelPosition.NONE;
    if (gamepad1.left_bumper && gamepad1.right_bumper) {
      driverChoice = PixelPosition.CENTER;
    } else if (gamepad1.left_bumper) {
      driverChoice = PixelPosition.LEFT;
    } else if (gamepad1.right_bumper) {
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
    if (gamepad2.left_bumper && gamepad2.right_bumper) {
      operatorChoice = PixelPosition.CENTER;
    } else if (gamepad2.left_bumper) {
      operatorChoice = PixelPosition.LEFT;
    } else if (gamepad2.right_bumper) {
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
      gamepad1.stopRumble();
      gamepad2.stopRumble();
      return driverChoice;
    } else {
      switch (driverChoice) {
        case NONE:
          gamepad2.stopRumble();
          break;
        case LEFT:
          gamepad2.rumble(1, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case CENTER:
          gamepad2.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case RIGHT:
          gamepad2.rumble(0, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
      }
      switch (operatorChoice) {
        case NONE:
          gamepad1.stopRumble();
          break;
        case LEFT:
          gamepad1.rumble(1, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case CENTER:
          gamepad1.rumble(1, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
        case RIGHT:
          gamepad1.rumble(0, 1, Gamepad.RUMBLE_DURATION_CONTINUOUS);
          break;
      }
      return PixelPosition.NONE;
    }
  }

  public PixelPosition getPosition(Gamepad gamepad) {
    PixelPosition superuserChoice = PixelPosition.NONE;
    if (gamepad.left_bumper && gamepad.right_bumper) {
      superuserChoice = PixelPosition.CENTER;
    } else if (gamepad.left_bumper) {
      superuserChoice = PixelPosition.LEFT;
    } else if (gamepad.right_bumper) {
      superuserChoice = PixelPosition.RIGHT;
    }
    return superuserChoice;
  }

  public void reset() {
    Drive(0, 0, 0);
    robot.lift.lower();
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
  public final double sensitivity = 0.15;
  public final double speedLimit = 0.2;
  public final double turnSpeedLimit = 0.1;

  /**How far the front of the robot should be from the board */
  public final double DISTANCE = 7;

  public void placePixel(Camera.AprilTagPosition position) {
    try {
      camera.resume();
      robot.lift.raise();
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
          Drive(currentDetection.ftcPose.x > 1 ? 0.1 : currentDetection.ftcPose.x < -1 ? -0.1 : 0,
                            currentDetection.ftcPose.range > 20 ? 0.1 : currentDetection.ftcPose.range < 18 ? -0.1 : 0,
                            currentDetection.ftcPose.yaw > 1 ? -0.1
                                    : currentDetection.ftcPose.yaw < -1 ? 0.1 : 0);
          */
          Drive(Range.clip(tag.ftcPose.x * sensitivity, -speedLimit, speedLimit),
              Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
              Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
        } else {
          for (Camera.AprilTag _tag : tags) {
            if (_tag.position != Camera.AprilTagPosition.CENTER) {
              tag = _tag;
            }
          }
          tag = tag == null ? tags.get(0) : tag;
          switch (position) {
            case LEFT:
              Drive(tag.position == Camera.AprilTagPosition.CENTER ? -0.2 : -0.3,
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
            case RIGHT:
              Drive(tag.position == Camera.AprilTagPosition.CENTER ? 0.2 : 0.3,
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
            case CENTER:
              Drive(tag.position == Camera.AprilTagPosition.LEFT ? 0.2 : -0.2,
                  Range.clip((tag.ftcPose.range - DISTANCE) * sensitivity, -speedLimit, speedLimit),
                  Range.clip(tag.ftcPose.yaw * -turnSensitivity, -turnSpeedLimit, turnSpeedLimit));
              break;
          }
        }
      } catch (Camera.CameraNotStreamingException e) {
        Drive(0, 0, 0);
        //Do nothing, the camera should be starting
      } catch (Camera.NoTagsFoundException e) {
        Drive(0, 0, 0);
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
    telemetry.addData("Lift", "Left: %d; Right: %d", Math.round(robot.leftRiser.getSpeed() * 100),
        Math.round(robot.rightRiser.getSpeed() * 100));
    telemetry.addData("Elbow", "Left: %d; Right: %d", Math.round(robot.leftElbow.getPosition() * 100),
        Math.round(robot.rightElbow.getPosition() * 100));

    if (gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
      superuser = gamepad1;
    } else if (gamepad2.a && gamepad2.b && gamepad2.x && gamepad2.y) {
      superuser = gamepad2;
    } else if (gamepad1.b && gamepad2.b) {
      superuser = null;
    }

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
    camera.telemetryAprilTag();
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
  }

  public double m1, m2, m3, m4;

  public void Drive(double x, double y, double z) {
    // r *= steeringMultiplier;
    m1 = Range.clip(y + x + z, -1, 1);
    m2 = Range.clip(y - x - z, -1, 1);
    m3 = Range.clip(y - x + z, -1, 1);
    m4 = Range.clip(y + x - z, -1, 1);
    // gamepad1.rumble((m1 + m3) / 2, (m2 + m4) / 2,
    // Gamepad.RUMBLE_DURATION_CONTINUOUS);
    /*
     * frontLeft.setVelocity(m1);
     * frontRight.setVelocity(m2);
     * rearLeft.setVelocity(m3);
     * rearRight.setVelocity(m4);
     */
    robot.frontLeft.setSpeed(m1);
    robot.frontRight.setSpeed(m2);
    robot.rearLeft.setSpeed(m3);
    robot.rearRight.setSpeed(m4);

  }
}
