package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name = "TestBot", group = "C")
// @Disabled
public class TestBot extends OpMode {
  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotorEx motor0 = null;
  private DcMotor motor1 = null;
  private DcMotorEx motor2 = null;
  private DcMotor motor3 = null;

  private CRServo servo0 = null;
  private Servo servo1 = null;
  private CRServo servo2 = null;
  private Servo servo3 = null;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    // Initialize the hardware variables. Note that the strings used here as
    // parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    motor0 = hardwareMap.get(DcMotorEx.class, "Motor_0");
    motor0.resetDeviceConfigurationForOpMode();
    motor0.setDirection(DcMotor.Direction.FORWARD);
    motor0.setPower(0);
    motor0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    motor1 = hardwareMap.get(DcMotor.class, "Motor_1");
    motor1.resetDeviceConfigurationForOpMode();
    motor1.setDirection(DcMotor.Direction.FORWARD);
    motor1.setPower(0);
    motor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    motor2 = hardwareMap.get(DcMotorEx.class, "Motor_2");
    motor2.resetDeviceConfigurationForOpMode();
    motor2.setDirection(DcMotor.Direction.FORWARD);
    motor2.setPower(0);
    motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    motor3 = hardwareMap.get(DcMotor.class, "Motor_3");
    motor3.resetDeviceConfigurationForOpMode();
    motor3.setDirection(DcMotor.Direction.FORWARD);
    motor3.setPower(0);
    motor3.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    servo0 = hardwareMap.get(CRServo.class, "Servo_0");
    servo0.resetDeviceConfigurationForOpMode();
    servo0.setPower(0);

    servo1 = hardwareMap.get(Servo.class, "Servo_1");
    servo1.resetDeviceConfigurationForOpMode();
    servo1.setPosition(0);

    servo2 = hardwareMap.get(CRServo.class, "Servo_2");
    servo2.resetDeviceConfigurationForOpMode();
    servo2.setPower(0);

    servo3 = hardwareMap.get(Servo.class, "Servo_3");
    servo3.resetDeviceConfigurationForOpMode();
    servo3.setPosition(0);

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
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Motor 0", (int) (motor0.getPower() * 100));
    telemetry.addData("Motor 1", (int) (motor1.getPower() * 100));
    telemetry.addData("Motor 2", (int) (motor3.getPower() * 100));
    telemetry.addData("Motor 3", (int) (motor2.getPower() * 100));
    motor0.setPower(-gamepad1.left_stick_y);
    motor1.setPower(gamepad1.left_stick_x);
    motor2.setPower(-gamepad1.right_stick_y);
    motor3.setPower(gamepad1.right_stick_x);

    telemetry.addData("Servo 0", (int) (servo0.getPower() * 100));
    if (gamepad1.dpad_up) {
      servo0.setPower(1);
    } else if (gamepad1.dpad_down) {
      servo0.setPower(-1);
    } else {
      servo0.setPower(0);
    }

    telemetry.addData("Servo 1", (int) (servo1.getPosition() * 100));
    if (gamepad1.dpad_right) {
      servo1.setPosition(1);
    } else if (gamepad1.dpad_left) {
      servo1.setPosition(0);
    }

    telemetry.addData("Servo 2", (int) (servo2.getPower() * 100));
    if (gamepad1.y) {
      servo2.setPower(1);
    } else if (gamepad1.a) {
      servo2.setPower(-1);
    } else {
      servo2.setPower(0);
    }

    telemetry.addData("Servo 3", (int) (servo3.getPosition() * 100));
    if (gamepad1.b) {
      servo3.setPosition(1);
    } else if (gamepad1.x) {
      servo3.setPosition(0);
    }
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
  }
}
