package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Test")
public class DriveTest extends LinearOpMode{
    public final static double driveFast = 1;

    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        double driveSpeed = 0.5;

        // Config motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        while (opModeIsActive()) {
            // Drive Code
            /****************************************************************
             Drive code -- basic mecanum drive with a variable to allow driver to
             slow down the speed for more controlled movement near game pieces
             ****************************************************************/

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(gamepad1.right_bumper) {
                driveSpeed = driveFast;
            }
            else {
                driveSpeed = 0.5;
            }

            leftFront.setPower(frontLeftPower * driveSpeed);
            leftBack.setPower(backLeftPower * driveSpeed);
            rightFront.setPower(frontRightPower * driveSpeed);
            rightBack.setPower(backRightPower * driveSpeed);
        }
    }
}
