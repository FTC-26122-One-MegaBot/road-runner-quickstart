package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private Servo claw;
    private CRServo intake;
    private DcMotor wrist;
    private DcMotor arm;
    public MecanumDrive drive;


    private Pose2d initialPose;

    public Robot(HardwareMap hardwareMap) {
        initialPose = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(DcMotor.class, "wrist");
        arm = hardwareMap.get(DcMotor.class, "arm");
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
            intake.setPower(0);
            return false;
        }
    }
    public void doStuffHighbarTest() {
        TrajectoryActionBuilder Highbar = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(20,0 ));
        Actions.runBlocking(new SequentialAction(
                Highbar.build(),
                new SleepAction(1.0),
                new Highbar(),
                new SleepAction(10.0))
        );


    }
    public class Highbar implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setTargetPosition(2350);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);

            return false;
        }
    }



}


