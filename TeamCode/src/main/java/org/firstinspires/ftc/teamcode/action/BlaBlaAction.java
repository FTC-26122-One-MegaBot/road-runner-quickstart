package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;

public class BlaBlaAction implements Action {

    private Servo wrist;

    public BlaBlaAction(Servo wrist) {
        this.wrist = wrist;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return false;
    }

}
