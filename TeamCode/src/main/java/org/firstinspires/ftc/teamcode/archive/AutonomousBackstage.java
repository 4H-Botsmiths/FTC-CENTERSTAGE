package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.classes.DCMotor;
import org.firstinspires.ftc.teamcode.classes.HDMotor;
import org.firstinspires.ftc.teamcode.classes.CoreMotor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.Thread;
import java.util.Timer;

@Autonomous(name = "Autonomous Backstage", group = "A", preselectTeleOp = "Teleop")
public class AutonomousBackstage extends OpMode {
  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private DCMotor frontLeft = null;
  private DCMotor frontRight = null;
  private DCMotor rearLeft = null;
  private DCMotor rearRight = null;

  private Servo trapdoor = null;
  private DCMotor intake = null;
  private DCMotor leftRiser = null;
  private DCMotor rightRiser = null;

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
    frontLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
    frontRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
    rearLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
    rearRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));
    intake = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"));
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"), DcMotor.Direction.REVERSE);
    rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"));

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
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    runtime.reset();
    // Robot will start facing away from backstage
    /*
     * Drive(-0.5, 0, 0); // Drive to the center of the field
     * Thread.sleep(1750); // How long it takes the robot to get to the center of
     * the field
     * 
     * Drive(0, 0, 0); // Briefly stop robot
     * Thread.sleep(500);
     * 
     * Drive(0, -0.5, 0); // Drive backstage
     * Thread.sleep(1500); // How long it takes the robot to get backstage
     * 
     * Drive(0, 0, 0); // Briefly stop robot
     * Thread.sleep(500);
     * 
     * Drive(0.25, 0, 0); // Controlled crash into the side of the field
     * Thread.sleep(4000); // How long it takes to hit the side of the field and
     * straighten
     * 
     * Drive(0, 0, 0); // Briefly stop robot
     * Thread.sleep(500);
     */

    Drive(0, -0.25, 0); // Controlled crash into the back of the field

  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    if (runtime.time() > 3.75) {// How long it takes to hit the back of the field and straighten
      Drive(0, 0, 0); // Stop robot
      trapdoor.setPosition(0);
      intake.setSpeed(-0.75);
    }

    // Tank Mode uses one stick to control each wheel.
    // - This requires no math, but it is hard to drive forward slowly and keep
    // straight.
    // leftPower = -gamepad1.left_stick_y ;
    // rightPower = -gamepad1.right_stick_y ;

    // Send calculated power to wheels
    // Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
    // Show the elapsed game time and wheel power.
    telemetry.addData("Status", "Running %f", (float) runtime.time());
    telemetry.addData("Front Left", Math.round(frontLeft.getSpeed() * 100));
    telemetry.addData("Front Right", Math.round(frontRight.getSpeed() * 100));
    telemetry.addData("Rear Right", Math.round(rearRight.getSpeed() * 100));
    telemetry.addData("Rear Left", Math.round(rearLeft.getSpeed() * 100));
    telemetry.addData("Intake", Math.round(rearLeft.getSpeed() * 100));
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
    frontLeft.setSpeed(m1);
    frontRight.setSpeed(m2);
    rearLeft.setSpeed(m3);
    rearRight.setSpeed(m4);

  }
}