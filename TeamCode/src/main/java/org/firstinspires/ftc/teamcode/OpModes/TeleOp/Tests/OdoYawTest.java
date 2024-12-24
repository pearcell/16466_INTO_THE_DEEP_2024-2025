package org.firstinspires.ftc.teamcode.OpModes.TeleOp.Tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class OdoYawTest extends LinearOpMode {

    private DcMotorEx leftPod;
    private DcMotorEx rightPod;

    private IMU imu;

    double trackWidth = 26.35;
    double ticksPerRev = 2000;
    double cmPerTick = (2 * Math.PI * 24) / ticksPerRev;
    int oldRightPosition;
    int oldLeftPosition;
    int currentRightPosition = 0;
    int currentLeftPosition = 0;

    double heading = 0;

    @Override
    public void runOpMode() {
        leftPod = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        rightPod = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();

        waitForStart();

        while (opModeIsActive()) {
            oldRightPosition = currentRightPosition;
            oldLeftPosition = currentLeftPosition;

            currentRightPosition = rightPod.getCurrentPosition();
            currentLeftPosition = -leftPod.getCurrentPosition();

            int dn1 = currentLeftPosition  - oldLeftPosition;
            int dn2 = currentRightPosition - oldRightPosition;

            double dTheta = cmPerTick * ((dn2-dn1) / (trackWidth));
            heading += dTheta * 180/Math.PI;

            telemetry.addData("yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("heading", heading);
            telemetry.update();
        }
    }
}
