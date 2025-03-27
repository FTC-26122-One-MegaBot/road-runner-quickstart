package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Mega TeleOp")
public class MegaTeleOp extends LinearOpMode {

    private Servo claw;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor arm;
    public final static double driveFast = 1;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;


    String currentState;
    boolean lastGrab;
    int targetArm;
    int targetWrist;
    String INTAKE;
    String LOW_BASKET;
    String INIT;
    boolean lastHook;
    String MANUAL;
    String HOVER;
    String HOVER_HIGH;
    String CLIP_HIGH;
    boolean clawOpen = false;
    double wristPower;
    double armPower;

    private Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        double driveSpeed = 0.5;
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "arm");
        MANUAL = "MANUAL";
        INTAKE = "INTAKE";
        HOVER_HIGH = "HOVER_HIGH";
        CLIP_HIGH = "CLIP_HIGH";
        LOW_BASKET = "LOW_BASKET";
        INIT = "INIT";
        HOVER = "HOVER";
        currentState = INIT;
        lastHook = false;
        lastGrab = false;



            waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here

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

                robot.updatePoseEstimate();
                GAMEPAD_INPUT_STATE();
                GAMEPAD_INPUT_TOGGLE();
                GAMEPAD_INPUT_MANUAL();
                GAMEPAD_INTAKE();
                STATE_MACHINE();
                Wrist();
                Arm();

                TELEMETRY(robot.drive);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_TOGGLE() {
        if (gamepad1.right_bumper) {
            claw.setPosition(0.5);
        } else {
            claw.setPosition(0.2);
        }
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_STATE() {
        if (gamepad1.a) {
            currentState = INTAKE;
        } else if (gamepad1.b){
            currentState = HOVER;
        } else if (gamepad1.y) {
            currentState = CLIP_HIGH;
        } else if (gamepad1.x) {
            currentState = LOW_BASKET;
        } else if (gamepad1.left_bumper) {
            currentState = INIT;
        }
        lastGrab = gamepad1.b;
        lastHook = gamepad1.y;
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INTAKE() {
        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    /**
     * Describe this function...
     */
    private void Wrist() {
        if (currentState.equals(CLIP_HIGH)) {
            wrist.setPower(0.2);
        } else {
            wrist.setPower(0.3);
        }
        wrist.setTargetPosition(targetWrist);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    /**
     * Describe this function...
     */
    private void Arm() {
        if (currentState.equals(CLIP_HIGH)) {
            arm.setPower(0.5);
        } else {
            arm.setPower(1);
        }
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_MANUAL() {
        if (gamepad1.dpad_up) {
            currentState = MANUAL;
            targetArm = targetArm <= 2800 ? targetArm + 25 : targetArm;
        } else if (gamepad1.dpad_down) {
            currentState = MANUAL;
            targetArm = targetArm >= 0 ? targetArm -25 : 0;
        } else if (gamepad1.dpad_left) {
            currentState = MANUAL;
            targetWrist = targetWrist <= 0 ? targetWrist + 5 : 0;
        } else if (gamepad1.dpad_right) {
            currentState = MANUAL;
            targetWrist = targetWrist >= -240 ? targetWrist -5 : -240;
        }
    }

    /**
     * Describe this function...
     */
    private void STATE_MACHINE() {
        if (currentState.equals(INIT)) {
            targetArm = 0;
            targetWrist = 0;
        } else if (currentState.equals(INTAKE)) {
            targetArm = 325 ;
            targetWrist = -235;
        } else if (currentState.equals(HOVER)) {
            targetArm = 500;
            targetWrist = -215;
        } else if (currentState.equals(CLIP_HIGH)) {
            targetArm = 2025;
            targetWrist = -245;
        } else if (currentState.equals(LOW_BASKET)) {
            targetArm = 2050;
            targetWrist = -240;
        } else {
            currentState = MANUAL;
        }
    }

    /**
     * Describe this function...
     */
    private void TELEMETRY(MecanumDrive drive) {

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.addData("right_bumper", gamepad1.right_bumper);
        telemetry.addData("STATE:", currentState);
        telemetry.addData("clawOpen", clawOpen ? "Open" : "Closed");
        telemetry.addData("Arm Position", arm.getCurrentPosition());
        telemetry.addData("Arm Power", arm.getPower());
        telemetry.addData("Target arm", targetArm);
        telemetry.addData("Wrist Position", wrist.getCurrentPosition());
        telemetry.addData("Wrist Power", wrist.getPower());
        telemetry.addData("Target wrist", targetWrist);
        telemetry.addData("Claw position", claw.getPosition());
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay().setStroke("#3F51B5");
        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
