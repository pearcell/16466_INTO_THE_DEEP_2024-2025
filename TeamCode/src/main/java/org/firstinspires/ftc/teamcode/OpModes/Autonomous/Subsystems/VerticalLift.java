package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalLift {
    public DcMotorEx frontLift;
    public DcMotorEx backLift;

    public VerticalLift(HardwareMap hardwareMap) {
        frontLift = hardwareMap.get(DcMotorEx.class, "frontLift");
        backLift = hardwareMap.get(DcMotorEx.class, "backLift");
        backLift.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
                frontLift.setPower(1);
                backLift.setPower(1);
                initialized = true;
            }

            double posFront = frontLift.getCurrentPosition();
            double posBack = backLift.getCurrentPosition();

            if (posFront < encoderVal || posBack < encoderVal) {
                return true;
            } else {
                frontLift.setPower(0);
                backLift.setPower(0);
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
                frontLift.setPower(-1);
                backLift.setPower(-1);
                initialized = true;
            }

            double posLeft = frontLift.getCurrentPosition();
            double posRight = backLift.getCurrentPosition();

            if (posLeft > encoderValue || posRight > encoderValue) {
                return true;
            } else {
                frontLift.setPower(0);
                backLift.setPower(0);
                return false;
            }
        }
    }

    public Action lower(int encVal1) {
        return new Lower(encVal1);
    }

}
