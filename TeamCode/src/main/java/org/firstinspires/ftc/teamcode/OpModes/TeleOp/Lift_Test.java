package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Lift_Test extends LinearOpMode{

    DcMotor frontLift;
    DcMotor backLift;
    @Override
    public void runOpMode() {
        frontLift = hardwareMap.dcMotor.get("frontLift");
        backLift = hardwareMap.dcMotor.get("backLift");
        backLift.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            frontLift.setPower(-gamepad1.left_trigger);
            backLift.setPower(-gamepad1.left_trigger);

            frontLift.setPower(gamepad1.right_trigger);
            backLift.setPower(gamepad1.right_trigger);
        }
    }
}
