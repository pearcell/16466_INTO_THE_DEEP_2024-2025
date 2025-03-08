package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class Lift_Test extends LinearOpMode{


    @Override
    public void runOpMode() {
        DcMotor frontLift = hardwareMap.dcMotor.get("frontLift");
        DcMotor backLift = hardwareMap.dcMotor.get("backLift");
       frontLift.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad2.left_trigger > 0) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                frontLift.setPower(-gamepad2.left_trigger);
                backLift.setPower(-gamepad2.left_trigger);
            }
            if (gamepad2.right_trigger > 0) {
                frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
                frontLift.setPower(gamepad2.right_trigger);
                backLift.setPower(gamepad2.right_trigger);
            }

        }
        telemetry.addData("right Trigger", gamepad1.right_trigger);
        telemetry.addData("left Trigger", gamepad1.left_trigger);
    }
}
