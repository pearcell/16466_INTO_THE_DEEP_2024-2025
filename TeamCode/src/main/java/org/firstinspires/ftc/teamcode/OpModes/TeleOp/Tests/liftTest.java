package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class liftTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLift = hardwareMap.dcMotor.get("frontLift");
        DcMotor backLift = hardwareMap.dcMotor.get("backLift");
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done

        waitForStart();
        while (opModeIsActive()) {

          /*  if (gamepad1.a) {
                frontLift.setPower(-0.5);
                backLift.setPower(0.5);
            } else if (backLift.getCurrentPosition() == 10000) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (frontLift.getCurrentPosition() == 10000) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }

            if (gamepad1.left_bumper) {
                frontLift.setPower(-0.5);
                backLift.setPower(0.5);
            } else if (backLift.getCurrentPosition() == 900) {
                frontLift.setPower(0);
                backLift.setPower(0);
            } else if (frontLift.getCurrentPosition() == 900) {
                frontLift.setPower(0);
                backLift.setPower(0);
            }*/


            if (gamepad1.right_trigger > 0) {
                frontLift.setPower(-gamepad1.right_trigger);
                backLift.setPower(gamepad1.right_trigger);
            } else {
                frontLift.setPower(0);
                backLift.setPower(0);
            }



        }
    }
}
