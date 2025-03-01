package org.firstinspires.ftc.teamcode.OpModes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@TeleOp
public class Motor_Test extends LinearOpMode {


    @Override
    public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");



        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                frontRightMotor.setPower(1);

            }
            if (gamepad1.b) {
                frontLeftMotor.setPower(1);
            }
            if (gamepad1.x) {
                backRightMotor.setPower(1);

            }
            if (gamepad1.y) {
                backLeftMotor.setPower(1);
            }


            telemetry.addData("frontLeft", frontLeftMotor.getPower());
            telemetry.addData("frontRight", frontRightMotor.getPower());
            telemetry.addData("backLeft", backLeftMotor.getPower());
            telemetry.addData("backRight", backRightMotor.getPower());


            telemetry.update();
        }
    }
}
