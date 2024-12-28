package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VerticalLift {
    private DcMotorEx leftLift;
    private DcMotorEx rightLift;
    int top = 1000;
    int bottom = 0;

    public VerticalLift(HardwareMap hardwareMap) {
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
    }

    public class Raise implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftLift.setPower(1);
                rightLift.setPower(1);
                initialized = true;
            }

            double posLeft = leftLift.getCurrentPosition();
            double posRight = rightLift.getCurrentPosition();

            if (posLeft < top || posRight < top) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                return false;
            }

        }
    }

    public Action raise() {
        return new Raise();
    }

    public class Lower implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                leftLift.setPower(-1);
                rightLift.setPower(-1);
                initialized = true;
            }

            double posLeft = leftLift.getCurrentPosition();
            double posRight = rightLift.getCurrentPosition();

            if (posLeft > bottom || posRight > bottom) {
                return true;
            } else {
                leftLift.setPower(0);
                rightLift.setPower(0);
                return false;
            }
        }
    }

    public Action lower() {
        return new Lower();
    }

}
