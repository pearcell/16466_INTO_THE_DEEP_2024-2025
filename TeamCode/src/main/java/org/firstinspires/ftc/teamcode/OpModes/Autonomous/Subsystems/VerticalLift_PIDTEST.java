package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VerticalLift_PIDTEST {
    public DcMotorEx frontLift;
    public DcMotorEx backLift;


    public VerticalLift_PIDTEST(HardwareMap hardwareMap) {
        frontLift = hardwareMap.get(DcMotorEx.class, "frontLift");
        backLift = hardwareMap.get(DcMotorEx.class, "backLift");
        backLift.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public class Raise implements Action {
        private boolean initialized = false;
        private int encoderVal;
        private Servo servoMain;
        double servoPosCurrent;
        double servoPosPrevious = 0;
        private PIDController pid = new PIDController();
        private ElapsedTime timer = new ElapsedTime();

        public Raise(int encVal, Servo servo) {
            encoderVal = encVal;
            servoMain = servo;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
            }
            servoPosCurrent = servoMain.getPosition();
            double posFront = frontLift.getCurrentPosition();
            double posBack = backLift.getCurrentPosition();

            //pid stuff here
            frontLift.setPower(pid.calculate(encoderVal, posFront, timer.seconds())); //front pid output
            backLift.setPower(pid.calculate(encoderVal, posBack, timer.seconds())); //back pid output


            if (Math.abs(servoPosCurrent - servoPosPrevious) >  0.05 && servoPosPrevious != 0) {
                return false;
            } else {
                servoPosPrevious = servoPosCurrent;
                timer.reset();
                return true;
            }
        }
    }

    public Action raise(int encVal1, Servo servo) {
        return new Raise(encVal1, servo);
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
