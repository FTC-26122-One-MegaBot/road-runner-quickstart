package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.HeadingPosePath;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
                .strafeTo(new Vector2d(0,-20));
//                        .strafeTo(new Vector2d(20, 20))
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));

        Actions.runBlocking(tab1.build());
    }

    public void doStuffL() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                        .strafeTo(new Vector2d(0,20));
//                        .strafeTo(new Vector2d(20, 20))
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));

        Actions.runBlocking(tab1.build());
    }
    public void doStuffTest() {
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(27.5,3));
//                .strafeTo(new Vector2d(88.45,3));
//                        .strafeTo(new Vector2d(20, 0))
//                        .strafeTo(new Vector2d(0,0));

        Actions.runBlocking(tab1.build());
    }
}
