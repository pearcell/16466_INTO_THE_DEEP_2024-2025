package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalExtensionOLD {
    public Servo servo;

    public HorizontalExtensionOLD(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "ServoArm");
    }

    public class Extend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(.4);
            return false;
        }
    }

    public Action extend() {
        return new Extend();
    }

    public class Retract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(.7);
            return false;
        }
    }

    public Action retract() {
        return new Retract();
    }


}
