package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        DcMotorEx motorL;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (motorL.getCurrentPosition() <= 1000) {
                clawServo.setPosition(openPos);
                return false;
            }
            return true;
        }

        public Drop(DcMotorEx motor) {
            motorL = motor;
        }
    }



    public Action drop(DcMotorEx motor) {
        return new Drop(motor);
    }
}
