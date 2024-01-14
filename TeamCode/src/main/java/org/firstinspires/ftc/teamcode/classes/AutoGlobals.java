package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoGlobals {
  public static final double PRIMARY_SPEED = 0.2;
  public static final double SECONDARY_SPEED = 0.15;
  public static final double TERTIARY_SPEED = 0.3;

  /** How long in milliseconds to delay before starting to move - make sure it still allows time to place on board and get backstage */
  public static final int DELAY_DURATION = 1000;
  public static final int STRAFE_DURATION = 3000;
  public static final int PLACE_DURATION = 2000;
  public static final int BACKUP_DURATION = 1000;
  public static final int PARKING_STRAFE_DURATION = 6000;
  public static final int PARKING_DURATION = 6000;

  public static final int CREEP_DURATION = 1000;

  /*Use commented code below as a template, but this function will not receive any input from the gamepads and will need to know that pixels have been placed 1000 milliseconds after entering the placing phase*/
  // degrees are more sensitive than inches, so they need finer control
  public static final double turnSensitivity = 0.025;
  public static final double sensitivity = 0.05;
  public static final double speedLimit = 0.15;
  public static final double turnSpeedLimit = 0.1;
  public static final double precisionSpeedLimit = 0.15;

  public final static double pressureSpeed = 0.1;

  public final static double tolerance = 5;
  public final static double precisionTolerance = 1.5;

  /**How far the front of the robot should be from the board */
  public final static double DISTANCE = 10; //6 final distance?
  public final static double FINAL_DISTANCE = 6;

  /** How far should the robot be from the side of the board before pulling forward to park */
  public final static double PARKING_X_DISTANCE = 3;

  /** How long the robot should strafe for to get to the center of the field  */
  public final static int ONSTAGE_STRAFE_DURATION = 4000;
  /** How long the robot should drive forward to get backstage */
  public final static int ONSTAGE_DRIVE_DURATION = 4000;

  public final static double POPUP_SPEED = 0.2;
  public final static int POPUP_DURATION = 500;

  public final static int LETS_BACK_UP = 6000;
}