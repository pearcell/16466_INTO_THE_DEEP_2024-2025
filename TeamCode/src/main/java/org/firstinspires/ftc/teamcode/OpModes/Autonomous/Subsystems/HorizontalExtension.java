package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalExtension {
    public Servo servo;

    public HorizontalExtension(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "ServoArm");
    }

    public class Extend implements Action {
        double pos3;

        public Extend(double pos2) {
            pos3 = pos2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(pos3);
            return false;
        }
    }


    public Action extend(double pos1) {
        return new Extend(pos1);
    }

    public class Retract implements Action {
        double posC;

        public Retract(double posB) {
            posC = posB;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(posC);
            return false;
        }
    }

    public Action retract(double posA) {
        return new Retract(posA);
    }


}
