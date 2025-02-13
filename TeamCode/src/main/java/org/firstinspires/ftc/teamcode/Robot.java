package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    private final Telemetry telemetry;
    private Servo claw;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor arm;
    public MecanumDrive drive;


    private Pose2d initialPose;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "arm");
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void setDrivePowers(PoseVelocity2d powers) {
        drive.setDrivePowers(powers);
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void doStuffR() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,-155));

//                        .strafeTo(new Vector2d(20, 20))
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));

        Actions.runBlocking(tab1.build());
    }

    public void doStuffL() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(0,40));
//                        .strafeTo(new Vector2d(20, 20))
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));

        Actions.runBlocking(tab1.build());
    }
    public void doStuffTest() {
        TrajectoryActionBuilder geel_blokje_1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0,15 ))
                .strafeTo(new Vector2d(18,15 ));
//                .strafeTo(new Vector2d(88.45,3));
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));
        TrajectoryActionBuilder geel_blokje_2 = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(20,15));

        Actions.runBlocking(new SequentialAction(
                geel_blokje_1.build(),
                new SleepAction(1.0),
                new Intake(),
                new SleepAction(1.0),
                new Grabbel_aan(),
                geel_blokje_2.build(),
                new SleepAction(1.0),
                new Grabbel_uit())
        );




    }

    public class Intake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetPosition(400);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            wrist.setTargetPosition(-240);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(0.3);

            return false;
        }
    }

    public class Grabbel_aan implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(1);
            return false;
        }
    }

    public class Grabbel_uit implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(-1);
            return false;
        }
    }
    public void doStuffHighbarTest() {
        TrajectoryActionBuilder highbar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20,0 ));

        TrajectoryActionBuilder highbar_Observzone = drive.actionBuilder(initialPose)
                //.splineTo(new Vector2d(-25, 0), Math.toRadians(0));
                .lineToX(-6)
                .turnTo(Math.toRadians(-45))
                .lineToY(-50)
                .lineToY(-40)
                .turnTo(Math.toRadians(-90));
        TrajectoryActionBuilder in_intake = drive.actionBuilder(initialPose)
                .lineToX(7);
        TrajectoryActionBuilder observe_highbar = drive.actionBuilder(initialPose)
                .lineToX(7)
                .turnTo(Math.toRadians(-45))
                .lineToY(-10)
                .turnTo(Math.toRadians(-45));



        Actions.runBlocking(new SequentialAction(
                new Highbar(),
                new SleepAction(2.0),
                highbar.build()
        ));
        telemetry.addData("heading bij sub",Math.toRadians(drive.pose.heading.toDouble()));
        telemetry.addData("x bij sub", drive.pose.position.x);
        telemetry.addData("y bij sub", drive.pose.position.y);
        telemetry.update();




        Actions.runBlocking(new SequentialAction(

                new INIT(),
                new SleepAction(1),
                highbar_Observzone.build(),
                new Grabbel_uit(),
                new SleepAction(3),
                new INTAKE(),
                new SleepAction(1),
                in_intake.build(),
                new SleepAction(0.2),
                new Grabbel_aan(),
                new SleepAction(1),
                new INIT(),
                observe_highbar.build(),
                new Highbar(),
                new SleepAction(2.0),
                highbar.build()

//                new SleepAction(1)
        ));

        telemetry.addData("heading bij observe",Math.toRadians(drive.pose.heading.toDouble()));
        telemetry.addData("x bij observe", drive.pose.position.x);
        telemetry.addData("y bij observe", drive.pose.position.y);
        telemetry.update();

        Actions.runBlocking(new SequentialAction(
                new SleepAction(10.0)
        ));

    }
    public class Highbar implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetPosition(2025);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5);
            wrist.setTargetPosition(-245);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(0.2);

            return false;
        }
    }


    public class INIT implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setTargetPosition(0);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(0.3);

            return false;
        }
    }

    public class INTAKE implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetPosition(325);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            wrist.setTargetPosition(-235);
            wrist.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wrist.setPower(0.3);

            return false;
        }
    }

}


