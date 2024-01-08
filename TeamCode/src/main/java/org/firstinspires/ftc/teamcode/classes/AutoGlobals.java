package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoGlobals {

  /** How long in milliseconds to delay before starting to move - make sure it still allows time to place on board and get backstage */
  public static final int DELAY_DURATION = 1000;
  public static final int STRAFE_DURATION = 1000;
  public static final int PLACE_DURATION = 3000;
  public static final int BACKUP_DURATION = 500;
  public static final int PARKING_STRAFE_DURATION = 1000;
  public static final int PARKING_DURATION = 1000;

  /*Use commented code below as a template, but this function will not receive any input from the gamepads and will need to know that pixels have been placed 1000 milliseconds after entering the placing phase*/
  // degrees are more sensitive than inches, so they need finer control
  public static final double turnSensitivity = 0.05;
  public static final double sensitivity = 0.075;
  public static final double speedLimit = 0.15;
  public static final double turnSpeedLimit = 0.1;
  public static final double precisionSpeedLimit = 0.15;

  public final static double pressureSpeed = 0.1;

  public final static double tolerance = 5;
  public final static double precisionTolerance = 1.5;

  /**How far the front of the robot should be from the board */
  public final static double DISTANCE = 10; //6 final distance?
  public final static double FINAL_DISTANCE = 6;
}