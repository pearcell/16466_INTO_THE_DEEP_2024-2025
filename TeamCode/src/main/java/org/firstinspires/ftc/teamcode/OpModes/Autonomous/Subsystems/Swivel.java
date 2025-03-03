package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Swivel {
    private Servo swivelServo;

    public Swivel(HardwareMap hardwareMap) {
        swivelServo = hardwareMap.get(Servo.class, "ServoClawRotate");
    }

    public class Turn implements Action {
        double pos3;

        public Turn(double pos2) {
            pos3 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            swivelServo.setPosition(pos3);
            return false;
        }
    }

    public Action turn(double pos1) {
        return new Turn(pos1);
    }
}
