package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarClaw {
    private Servo clawServo;
    private double openPos = 0.5;
    private double closedPos = 0;

    public FourBarClaw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "ServoClawGrab");
    }

    public class Grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(closedPos);
            return false;
        }
    }

    public Action grab() {
        return new FourBarClaw.Grab();
    }

    public class Drop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(openPos);
            return false;

        }
    }

    public Action drop() {
        return new FourBarClaw.Drop();
    }
}
