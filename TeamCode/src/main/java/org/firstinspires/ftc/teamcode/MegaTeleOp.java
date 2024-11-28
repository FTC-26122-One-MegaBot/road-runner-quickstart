package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.action.BlaBlaAction;

@TeleOp(name = "Mega TeleOp")
public class MegaTeleOp extends LinearOpMode {

    private Servo claw;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor arm;

    String currentState;
    boolean lastGrab;
    int targetArm;
    int targetWrist;
    String INTAKE;
    String LOW_BASKET;
    String INIT;
    boolean lastHook;
    String MANUAL;
    String WALL_GRAB;
    String WALL_UNHOOK;
    String HOVER_HIGH;
    String CLIP_HIGH;
    boolean clawOpen = false;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "arm");
        MANUAL = "MANUAL";
        INTAKE = "INTAKE";
        WALL_GRAB = "WALL_GRAB";
        WALL_UNHOOK = "WALL_UNHOOK";
        HOVER_HIGH = "HOVER_HIGH";
        CLIP_HIGH = "CLIP_HIGH";
        LOW_BASKET = "LOW_BASKET";
        INIT = "INIT";
        currentState = INIT;
        lastHook = false;
        lastGrab = false;

        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x
                ));

                drive.updatePoseEstimate();

                GAMEPAD_INPUT_STATE();
                GAMEPAD_INPUT_TOGGLE();
                GAMEPAD_INPUT_MANUAL();
                GAMEPAD_INTAKE();
                STATE_MACHINE();
                Wrist();
                Arm();
                TELEMETRY(drive);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_TOGGLE() {
        if (gamepad1.right_bumper) {
            claw.setPosition(0.17);
        } else {
            claw.setPosition(0.05);
        }
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_STATE() {
        if (gamepad1.a) {
            currentState = INTAKE;
        } else if (gamepad1.b && !lastGrab) {
            if (currentState.equals(WALL_GRAB)) {
                currentState = WALL_UNHOOK;
            } else {
                currentState = WALL_GRAB;
            }
        } else if (gamepad1.y && !lastHook) {
            if (currentState.equals(HOVER_HIGH)) {
                currentState = CLIP_HIGH;
            } else {
                currentState = HOVER_HIGH;
            }
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
        wrist.setTargetPosition(targetWrist);
        wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPower(0.3);
    }

    /**
     * Describe this function...
     */
    private void Arm() {
        arm.setTargetPosition(targetArm);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1);
    }

    /**
     * Describe this function...
     */
    private void GAMEPAD_INPUT_MANUAL() {
        if (gamepad1.dpad_up) {
            currentState = MANUAL;
            targetArm += 25;
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
            targetArm = 425 ;
            targetWrist = -240;
        } else if (currentState.equals(WALL_GRAB)) {
            targetArm = 1150;
            targetWrist = 0;
        } else if (currentState.equals(WALL_UNHOOK)) {
            targetArm = 1410;
            targetWrist = 0;
        } else if (currentState.equals(HOVER_HIGH)) {
            targetArm = 2150;
            targetWrist = 0;
        } else if (currentState.equals(CLIP_HIGH)) {
            targetArm = 2350;
            targetWrist = 0;
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
