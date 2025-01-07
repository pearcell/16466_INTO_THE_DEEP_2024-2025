package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalExtension {
    public Servo servo;

    public HorizontalExtension(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "ServoArm");
    }

    public class Extend implements Action {
        private DcMotorEx motorRef;
        private int encoderValue;

        public Extend(DcMotorEx motor, int encB) {
            motorRef = motor;
            encoderValue = encB;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (motorRef.getCurrentPosition() <= encoderValue) {
                servo.setPosition(.2);
                return false;
            }
            return true;
        }
    }

    public Action extend(DcMotorEx motor, int encA) {
        return new Extend(motor, encA);
    }

    public class Retract implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(.48);
            return false;
        }
    }

    public Action retract() {
        return new Retract();
    }


}
