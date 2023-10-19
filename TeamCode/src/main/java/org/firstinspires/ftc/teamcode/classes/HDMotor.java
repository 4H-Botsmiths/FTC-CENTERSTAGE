package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class HDMotor implements DcMotorEx {
  private final int TICKS_PER_ROTATION = 28;

  public void setSpeed(double speed) {
    setVelocity(speed * TICKS_PER_ROTATION);
  }

  public double getSpeed() {
    return getVelocity() / TICKS_PER_ROTATION;
  }
}