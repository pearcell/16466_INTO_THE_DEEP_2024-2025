package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {
    private Servo clawServo;
    private int openPos;
    private int closedPos;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public class Grab implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                clawServo.setPosition(closedPos);
                initialized = true;
            }
            return clawServo.getPosition() < closedPos;
        }
    }

    public Action grab() {
        return new Grab();
    }

    public class Drop implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                clawServo.setPosition(openPos);
                initialized = true;
            }
            return clawServo.getPosition() < openPos;
        }
    }

    public Action drop() {
        return new Drop();
    }
}
