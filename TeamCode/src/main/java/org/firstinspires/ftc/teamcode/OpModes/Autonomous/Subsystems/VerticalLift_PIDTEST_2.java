package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VerticalLift_PIDTEST_2 {
    public DcMotorEx frontLift;
    public DcMotorEx backLift;
    public double targetPos;
    double f = .15;


    public VerticalLift_PIDTEST_2(HardwareMap hardwareMap) {
        frontLift = hardwareMap.get(DcMotorEx.class, "frontLift");
        backLift = hardwareMap.get(DcMotorEx.class, "backLift");
        backLift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public class Move implements Action {
        private boolean initialized = false;
        private PIDController pid = new PIDController(0.002,0,0);
        private ElapsedTime timer = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
            }
            double posFront = frontLift.getCurrentPosition();
            double posBack = backLift.getCurrentPosition();

            //pid stuff here
            frontLift.setPower(pid.calculate(targetPos, posFront, timer.seconds()) + f); //front pid output
            backLift.setPower(pid.calculate(targetPos, posBack, timer.seconds()) + f); //back pid output


            return true;
        }
    }

    public Action move() {
        return new Move();
    }

    public class setSlidePos implements Action {
        double posFinal;

        public setSlidePos(double posBetween) {
            posFinal = posBetween;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetPos = posFinal;
            return false;
        }
    }

    public Action SetSlidePos(double posInitial) {
        return new setSlidePos(posInitial);
    }
}
