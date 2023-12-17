package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.FutureTask;

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
  public DCMotor intake = null;
  /** Port 2.1 ? */
  public DCMotor leftRiser = null;
  /** Port 2.2 ? */
  public DCMotor rightRiser = null;
  /** Port 0 */
  public Servo trapdoor = null;
  /** Port 1 ?  */
  public Servo leftElbow = null;
  /** Port 2 ? */
  public Servo rightElbow = null;

  public Lift lift = null;

  public Robot(HardwareMap hardwareMap) {
    voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

    frontLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
    frontRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "FrontRight"), DcMotor.Direction.REVERSE);
    rearLeft = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearLeft"));
    rearRight = new DCMotor(hardwareMap.get(DcMotorEx.class, "RearRight"));

    intake = new DCMotor(hardwareMap.get(DcMotorEx.class, "Intake"), DcMotor.Direction.REVERSE);
    leftRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "LeftRiser"));
    rightRiser = new DCMotor(hardwareMap.get(DcMotorEx.class, "RightRiser"), DcMotor.Direction.REVERSE);
    trapdoor = hardwareMap.get(Servo.class, "Trapdoor");
    trapdoor.setDirection(Servo.Direction.REVERSE);
    leftElbow = hardwareMap.get(Servo.class, "LeftElbow");
    rightElbow = hardwareMap.get(Servo.class, "RightElbow");
    leftElbow.setDirection(Servo.Direction.REVERSE);
    leftElbow.scaleRange(0.03, 0.4);
    rightElbow.scaleRange(0.03, 0.4);

    lift = new Lift(leftRiser, rightRiser);

    CompletableFuture.runAsync(() -> {
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
    });
  }

  public double averageVoltage = 12;
  public double averagePerformance = 100;
  public double performance = 100;

  public enum LiftStatus {
    RAISED, RAISING, LOWERED, LOWERING
  }

  public class Lift {
    DCMotor leftMotor;
    DCMotor rightMotor;

    public Lift(DCMotor left, DCMotor right) {
      this.leftMotor = left;
      this.rightMotor = right;
    }

    public void setSpeed(double speed) {
      if (status == LiftStatus.RAISED || status == LiftStatus.LOWERED) {
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(speed);
      }
    }

    public LiftStatus status = LiftStatus.LOWERED;
    private ElapsedTime liftClock = new ElapsedTime();
    private final int DURATION = 1750;

    private double calcSpeed(double time) {
      return Range.clip(time < 0.5 * DURATION ? 1 : (DURATION - time) / DURATION, 0, 1);
    }

    private double calcSpeed(double time, double duration) {
      return Range.clip(time < 0.5 * duration ? 1 : (duration - time) / duration, 0, 1);
    }

    public void raise() {
      if (status == LiftStatus.RAISED || status == LiftStatus.RAISING) {
        return;
      }
      if (status == LiftStatus.LOWERING) {
        double raiseTime = liftClock.milliseconds();
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
              // Handle interrupted exception
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
          while (liftClock.milliseconds() < DURATION && status == LiftStatus.RAISING) {
            double time = liftClock.milliseconds();
            double speed = calcSpeed(time);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              // Handle interrupted exception
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

    public void lower() {
      if (status == LiftStatus.LOWERED || status == LiftStatus.LOWERING) {
        return;
      }
      if (status == LiftStatus.RAISING) {
        double lowerTime = liftClock.milliseconds();
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
              // Handle interrupted exception
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
          while (liftClock.milliseconds() < DURATION && status == LiftStatus.LOWERING) {
            double time = liftClock.milliseconds();
            double speed = -calcSpeed(time);
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
            try {
              Thread.sleep(20); // Sleep for 20 milliseconds
            } catch (InterruptedException e) {
              // Handle interrupted exception
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
  }
}