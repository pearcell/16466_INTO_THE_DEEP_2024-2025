package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo clawServo;
    private int openPos;
    private int closedPos;

    public Claw(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public class Grab implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            clawServo.setPosition(closedPos);
            return false;
        }
    }

    public Action grab() {
        return new Grab();
    }

    public class Drop implements Action {
        private DcMotorEx motorL;
        private int encoderValue;

        public Drop(DcMotorEx motor, int encB) {
            motorL = motor;
            encoderValue = encB;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (motorL.getCurrentPosition() >= encoderValue) {
                clawServo.setPosition(openPos);
                return false;
            }
            return true;
        }
    }



    public Action drop(DcMotorEx motor, int encA) {
        return new Drop(motor, encA);
    }
}
