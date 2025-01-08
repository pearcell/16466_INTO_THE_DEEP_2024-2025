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
    }

    public class Raise implements Action {
        private boolean initialized = false;
        private int encoderVal;
        private double powerR;

        public Raise(int encVal, double power2) {
            encoderVal = encVal;
            powerR = power2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                frontLift.setPower(powerR);
                backLift.setPower(powerR);
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

    public Action raise(int encVal1, double power1) {
        return new Raise(encVal1, power1);
    }

    public class Lower implements Action {
        private boolean initialized = false;
        private int encoderValue;
        private double powerL;

        public Lower(int encVal, double power2) {
            encoderValue = encVal;
            powerL = power2;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                frontLift.setPower(-powerL);
                backLift.setPower(-powerL);
                initialized = true;
            }

            double posFront = frontLift.getCurrentPosition();
            double posBack = backLift.getCurrentPosition();

            if (posFront > encoderValue || posBack > encoderValue) {
                return true;
            } else {
                frontLift.setPower(0);
                backLift.setPower(0);
                return false;
            }
        }
    }

    public Action lower(int encVal1, double power1) {
        return new Lower(encVal1, power1);
    }

}
