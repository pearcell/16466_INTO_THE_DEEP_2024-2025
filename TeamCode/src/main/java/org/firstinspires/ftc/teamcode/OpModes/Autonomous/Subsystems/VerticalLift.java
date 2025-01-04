package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalLift {
    public DcMotorEx leftLift;
    private DcMotorEx rightLift;

    public VerticalLift(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
    }

    public class Raise implements Action {
        private boolean initialized = false;
        private int encoderVal;

        public Raise(int encVal) {
            encoderVal = encVal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftLift.setPower(1);
                rightLift.setPower(1);
                initialized = true;
            }

            double posLeft = leftLift.getCurrentPosition();
            double posRight = rightLift.getCurrentPosition();

            if (posLeft < encoderVal || posRight < encoderVal) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                return false;
            }
        }
    }

    public Action raise(int encVal1) {
        return new Raise(encVal1);
    }

    public class Lower implements Action {
        private boolean initialized = false;
        private int encoderValue;

        public Lower(int encVal) {
            encoderValue = encVal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftLift.setPower(-1);
                rightLift.setPower(-1);
                initialized = true;
            }

            double posLeft = leftLift.getCurrentPosition();
            double posRight = rightLift.getCurrentPosition();

            if (posLeft > encoderValue || posRight > encoderValue) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                return false;
            }
        }
    }

    public Action lower(int encVal1) {
        return new Lower(encVal1);
    }

}
