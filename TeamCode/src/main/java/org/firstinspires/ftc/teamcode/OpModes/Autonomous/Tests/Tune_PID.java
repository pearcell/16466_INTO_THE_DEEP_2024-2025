package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.PIDController;

@Config
@TeleOp
public class Tune_PID extends LinearOpMode {

    public DcMotorEx frontLift;
    public DcMotorEx backLift;
    public static double targetPos;

    @Override
    public void runOpMode() {

        frontLift = hardwareMap.get(DcMotorEx.class, "frontLift");
        backLift = hardwareMap.get(DcMotorEx.class, "backLift");
        backLift.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDController pid = new PIDController();
        ElapsedTime timer = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            double posFront = frontLift.getCurrentPosition();
            double posBack = backLift.getCurrentPosition();

            //pid stuff here
            frontLift.setPower(pid.calculate(targetPos, posFront, timer.seconds())); //front pid output
            backLift.setPower(pid.calculate(targetPos, posBack, timer.seconds())); //back pid output

            telemetry.addData("frontPos", posFront);
            telemetry.addData("target", targetPos);
            telemetry.update();
        }
    }
}
