package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

@TeleOp(name = "BaseBot", group = "B")
// @Disabled
public class BaseBot extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx rearLeft = null;
    private DcMotorEx rearRight = null;

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
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontLeft.resetDeviceConfigurationForOpMode();
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setPower(0);
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontRight.resetDeviceConfigurationForOpMode();
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setPower(0);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeft = hardwareMap.get(DcMotorEx.class, "RearLeft");
        rearLeft.resetDeviceConfigurationForOpMode();
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setPower(0);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearRight = hardwareMap.get(DcMotorEx.class, "RearRight");
        rearRight.resetDeviceConfigurationForOpMode();
        rearRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setPower(0);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        telemetry.addData("Front Left PIDF", "");
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used. The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep
        // straight.
        // leftPower = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        // Drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front Left", frontLeft.getVelocity());
        telemetry.addData("Front Right", frontRight.getVelocity());
        telemetry.addData("Rear Right", rearRight.getVelocity());
        telemetry.addData("Rear Left", rearLeft.getVelocity());
        /*
         * telemetry.addData("Front", "(%.2f) %S|-|%S (%.2f)",
         * frontLeft.getPower(), (frontLeft.getPower() == 0 ? '-' : frontLeft.getPower()
         * > 0 ? '^' : 'v'),
         * frontRight.getPower(), (frontRight.getPower() == 0 ? '-' :
         * frontRight.getPower() > 0 ? '^' : 'v'));
         * telemetry.addData("Rear", "(%.2f) %S|-|%S (%.2f)",
         * rearLeft.getPower(), (rearLeft.getPower() == 0 ? '-' : rearLeft.getPower() >
         * 0 ? '^' : 'v'),
         * rearRight.getPower(), (rearRight.getPower() == 0 ? '-' : rearRight.getPower()
         * > 0 ? '^' : 'v'));
         */
        if (gamepad1.a) {
            frontLeft.setPower(0.1);
            frontRight.setPower(0.1);
            rearLeft.setPower(0.1);
            rearRight.setPower(0.1);
        } else {
            Drive(Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.left_stick_y, 3),
                    Math.pow(gamepad1.right_stick_x, 3));
        }
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
        frontLeft.setPower(m1);
        telemetry.addData("Front Left Set", (int) (m1 * 100));
        telemetry.addData("Front Left Get", (int) (frontLeft.getPower() * 100));
        frontRight.setPower(m2);
        telemetry.addData("Front Right Set", (int) (m2 * 100));
        telemetry.addData("Front Right Get", (int) (frontRight.getPower() * 100));
        rearLeft.setPower(m3);
        telemetry.addData("Rear Left Set", (int) (m3 * 100));
        telemetry.addData("Rear Left Get", (int) (rearLeft.getPower() * 100));
        rearRight.setPower(m4);
        telemetry.addData("Rear Rear Set", (int) (m4 * 100));
        telemetry.addData("Rear Right Get", (int) (rearRight.getPower() * 100));

    }
}
