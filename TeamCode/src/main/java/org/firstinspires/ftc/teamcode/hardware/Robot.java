package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.FutureTask;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.DCMotor;

public class Robot {
  public VoltageSensor voltage = null;

  /** Port 2.0 ? */
  public DCMotor frontLeft = null;
  /** Port 1 */
  public DCMotor frontRight = null;
  /** Port 2 */
  public DCMotor rearLeft = null;
  /** Port 3 */
  public DCMotor rearRight = null;

  /** Port 0 */
  private DCMotor intakeSpinner = null;
  /** Port 2.1 ? */
  private DCMotor leftRiser = null;
  /** Port 2.2 ? */
  private DCMotor rightRiser = null;
  /** Port 0 */
  private Servo trapdoor = null;
  /** Port 1 ?  */
  private Servo leftElbow = null;
  /** Port 2 ? */
  private Servo rightElbow = null;

  public Lift lift = null;
  public Intake intake = null;

  public Robot(HardwareMap hardwareMap) {
    voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

    frontLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
    frontRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
    rearLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
    rearRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));

    intakeSpinner = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"), DcMotor.Direction.REVERSE);
    leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"));
    rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"), DcMotor.Direction.REVERSE);
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    trapdoor.setDirection(Servo.Direction.REVERSE);
    //trapdoor.scaleRange(0.5, 0.8);

    leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
    rightElbow = hardwareMap.get(Servo.class, "RightElbow");
    rightElbow.setDirection(Servo.Direction.REVERSE);
    // leftElbow.scaleRange(0.7, 1);
    //rightElbow.scaleRange(0, 0.3);

    lift = new Lift(leftRiser, rightRiser, leftElbow, rightElbow);
    intake = new Intake(intakeSpinner, trapdoor);

    /*CompletableFuture.runAsync(() -> {
      while (true) {
        double currentVoltage = voltage.getVoltage();
        averageVoltage = (currentVoltage + averageVoltage) / 2;
        performance = (currentVoltage / averageVoltage);
        averagePerformance = (performance + averagePerformance) / 2;
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          //Do Nothing
        }
      }
    });*/
  }

  public double averageVoltage = 12;
  public double averagePerformance = 100;
  public double performance = 100;

  public double m1, m2, m3, m4;
  private double[] speeds = new double[] { 0, 0, 0 };

  /**
   * Drive the robot by setting the speeds of the four motors based on the input values.
   *
   * @param  x  the value of the x-axis input, ranging from -1 to 1
   * @param  y  the value of the y-axis input, ranging from -1 to 1
   * @param  z  the value of the z-axis input, ranging from -1 to 1
   */
  public void Drive(double x, double y, double z) {
    speeds[0] = x;
    speeds[1] = y;
    speeds[2] = z;
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

  /**
   * Drives the vehicle at a specified factor of the current speeds.
   *
   * @param  factor  the factor to multiply the current speeds by
   */
  public void Drive(double factor) {
    double x, y, z;
    x = speeds[0] * factor;
    y = speeds[1] * factor;
    z = speeds[2] * factor;
    m1 = Range.clip(y + x + z, -1, 1);
    m2 = Range.clip(y - x - z, -1, 1);
    m3 = Range.clip(y - x + z, -1, 1);
    m4 = Range.clip(y + x - z, -1, 1);
    frontLeft.setSpeed(m1);
    frontRight.setSpeed(m2);
    rearLeft.setSpeed(m3);
    rearRight.setSpeed(m4);
  }

  public class Intake {
    DCMotor motor;
    Servo servo;

    public void setPosition(double position) {
      position = Range.scale(position, 0, 1, 0.2, 0.5);
      servo.setPosition(position);
    }

    public Intake(DCMotor motor, Servo servo) {
      this.motor = motor;
      this.servo = servo;
    }

    public void hold() {
      setPosition(0);
      motor.setSpeed(0.5);
    }

    public void drop() {
      setPosition(1);
      motor.setSpeed(0.2);
    }

    public void grab() {
      setPosition(0.25);
      motor.setSpeed(1);
    }

    public void eject() {
      setPosition(0);
      motor.setSpeed(-1);
    }

    public void telemetries(Telemetry telemetry) {
      telemetry.addData("Intake", Math.round(motor.getSpeed() * 100));
      telemetry.addData("Trapdoor", Math.round(servo.getPosition() * 100));
    }

  }

  public enum LiftStatus {
    RAISED, RAISING, LOWERED, LOWERING
  }

  public class Lift {
    DCMotor leftMotor;
    DCMotor rightMotor;
    Servo leftServo;
    Servo rightServo;

    public Lift(DCMotor leftMotor, DCMotor rightMotor, Servo leftServo, Servo rightServo) {
      this.leftMotor = leftMotor;
      this.rightMotor = rightMotor;
      this.leftServo = leftServo;
      this.rightServo = rightServo;
      leftMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightMotor.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed) {
      if (status == LiftStatus.RAISED || status == LiftStatus.LOWERED) {
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(speed);
      }
    }

    public void setPosition(double position) {
      position = Range.scale(position, 0, 1, 0.1, 1);
      leftServo.setPosition(position);
      rightServo.setPosition(position);
    }

    public LiftStatus status = LiftStatus.LOWERED;
    private ElapsedTime liftClock = new ElapsedTime();
    private final int RAISE_DURATION = 1000; //Was 1500 - decreased for security until new speed curve tested
    private final int LOWER_DURATION = 750;//Was 1250 - decreased for security until new speed curve tested

    private double calcSpeed(double time, double duration) {
      if (time < 0.25 * duration) {
        return Range.clip(time / duration * 0.5, 0, 1);
      } else if (time > 0.75 * duration) {
        return Range.clip((duration - time) / duration * 0.5, 0, 1);
      } else {
        return 1;
      }
      //Old method: return Range.clip(time < 0.5 * duration ? 1 : (duration - time) / duration, 0, 1);
    }

    @Deprecated //Use `expand()` function instead
    public void raise() {
      if (status == LiftStatus.RAISED || status == LiftStatus.RAISING) {
        return;
      }
      if (status == LiftStatus.LOWERING) {
        double raiseTime = liftClock.milliseconds() + 500;
        status = LiftStatus.RAISING;
        liftClock.reset();
        CompletableFuture.runAsync(() -> {
          while (liftClock.milliseconds() < raiseTime && status == LiftStatus.RAISING) {
            double time = liftClock.milliseconds();
            double speed = calcSpeed(time, raiseTime);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              leftMotor.setSpeed(0);
              rightMotor.setSpeed(0);
              Thread.currentThread().interrupt();
            }
          }
          if (status == LiftStatus.RAISING) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            status = LiftStatus.RAISED;
          }
        });
      } else {
        status = LiftStatus.RAISING;
        liftClock.reset();
        CompletableFuture.runAsync(() -> {
          while (liftClock.milliseconds() < RAISE_DURATION && status == LiftStatus.RAISING) {
            double time = liftClock.milliseconds();
            double speed = calcSpeed(time, RAISE_DURATION);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              leftMotor.setSpeed(0);
              rightMotor.setSpeed(0);
              Thread.currentThread().interrupt();
            }
          }
          if (status == LiftStatus.RAISING) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            status = LiftStatus.RAISED;
          }
        });
      }
    }

    @Deprecated //Use `constrict()` function instead
    public void lower() {
      if (status == LiftStatus.LOWERED || status == LiftStatus.LOWERING) {
        return;
      }
      if (status == LiftStatus.RAISING) {
        double lowerTime = liftClock.milliseconds() + 500;
        status = LiftStatus.LOWERING;
        liftClock.reset();
        CompletableFuture.runAsync(() -> {
          while (liftClock.milliseconds() < lowerTime && status == LiftStatus.LOWERING) {
            double time = liftClock.milliseconds();
            double speed = -calcSpeed(time, lowerTime);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              leftMotor.setSpeed(0);
              rightMotor.setSpeed(0);
              Thread.currentThread().interrupt();
            }
          }
          if (status == LiftStatus.LOWERING) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            status = LiftStatus.LOWERED;
          }
        });
      } else {
        status = LiftStatus.LOWERING;
        liftClock.reset();
        CompletableFuture.runAsync(() -> {
          while (liftClock.milliseconds() < LOWER_DURATION && status == LiftStatus.LOWERING) {
            double time = liftClock.milliseconds();
            double speed = -calcSpeed(time, LOWER_DURATION);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              leftMotor.setSpeed(0);
              rightMotor.setSpeed(0);
              Thread.currentThread().interrupt();
            }
          }
          if (status == LiftStatus.LOWERING) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            status = LiftStatus.LOWERED;
          }
        });
      }
    }

    public void expand() {
      if (status != LiftStatus.LOWERED) {
        return;
      }
      status = LiftStatus.RAISING;
      liftClock.reset();
      CompletableFuture.runAsync(() -> {
        while (liftClock.milliseconds() < RAISE_DURATION && status == LiftStatus.RAISING) {
          double time = liftClock.milliseconds();
          double speed = calcSpeed(time, RAISE_DURATION);
          if (time > 0.5 * RAISE_DURATION) {
            setPosition(1);
          }
          leftMotor.setSpeed(speed);
          rightMotor.setSpeed(speed);
          try {
            Thread.sleep(20); // Sleep for 20 milliseconds
          } catch (InterruptedException e) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            Thread.currentThread().interrupt();
          }
        }
        if (status == LiftStatus.RAISING) {
          leftMotor.setSpeed(0);
          rightMotor.setSpeed(0);
          status = LiftStatus.RAISED;
        }
      });
    }

    /**How long to raise the lift after lowering (to reduce ground friction) */
    private final int POPUP_DURATION = 75;

    public void constrict() {
      if (status != LiftStatus.RAISED) {
        return;
      }
      status = LiftStatus.LOWERING;
      liftClock.reset();
      CompletableFuture.runAsync(() -> {
        setPosition(0);
        while (liftClock.milliseconds() < LOWER_DURATION && status == LiftStatus.LOWERING) {
          double time = liftClock.milliseconds();
          double speed = -calcSpeed(time, LOWER_DURATION);
          leftMotor.setSpeed(speed);
          rightMotor.setSpeed(speed);
          try {
            Thread.sleep(20); // Sleep for 20 milliseconds
          } catch (InterruptedException e) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            Thread.currentThread().interrupt();
          }
        }
        if (status == LiftStatus.LOWERING) {
          leftMotor.setSpeed(0.5);
          rightMotor.setSpeed(0.5);
          try {
            Thread.sleep(POPUP_DURATION);
          } catch (InterruptedException e) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            Thread.currentThread().interrupt();
          }
          if (status == LiftStatus.LOWERING) {
            leftMotor.setSpeed(0);
            rightMotor.setSpeed(0);
            status = LiftStatus.LOWERED;
          }
        }
      });
    }

    public void telemetries(Telemetry telemetry) {
      telemetry.addData("Lift", "Left: %d; Right: %d", Math.round(leftMotor.getSpeed() * 100),
          Math.round(rightMotor.getSpeed() * 100));
      telemetry.addData("Elbow", "Left: %d; Right: %d", Math.round(leftServo.getPosition() * 100),
          Math.round(rightServo.getPosition() * 100));
    }
  }

}