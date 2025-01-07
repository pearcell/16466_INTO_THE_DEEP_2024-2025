package org.firstinspires.ftc.teamcode.OpModes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.List;

@TeleOp
public class Motor_Test extends LinearOpMode {

    DcMotorEx Motor0;
    DcMotorEx Motor1;
    DcMotorEx Motor2;
    DcMotorEx Motor3;

    double current0;
    double current1;
    double current2;
    double current3;

    double velo0;
    double velo1;
    double velo2;
    double velo3;

    double inputVoltage;

    @Override
    public void runOpMode() {
        Motor0 = hardwareMap.get(DcMotorEx.class, "Motor0");
        Motor1 = hardwareMap.get(DcMotorEx.class, "Motor1");
        Motor2 = hardwareMap.get(DcMotorEx.class, "Motor2");
        Motor3 = hardwareMap.get(DcMotorEx.class, "Motor3");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                Motor0.setPower(1);
                Motor1.setPower(1);
                Motor2.setPower(1);
                Motor3.setPower(1);
            } else {
                Motor0.setPower(0);
                Motor1.setPower(0);
                Motor2.setPower(0);
                Motor3.setPower(0);
            }

            current0 = Motor0.getCurrent(CurrentUnit.AMPS);
            current1 = Motor1.getCurrent(CurrentUnit.AMPS);
            current2 = Motor2.getCurrent(CurrentUnit.AMPS);
            current3 = Motor3.getCurrent(CurrentUnit.AMPS);

            velo0 = Motor0.getVelocity();
            velo1 = Motor1.getVelocity();
            velo2 = Motor2.getVelocity();
            velo3 = Motor3.getVelocity();

            for(LynxModule hub : allHubs) {
                inputVoltage = hub.getInputVoltage(VoltageUnit.VOLTS);
            }

            telemetry.addData("0_Current", current0);
            telemetry.addData("1_Current", current1);
            telemetry.addData("2_Current", current2);
            telemetry.addData("3_Current", current3);

            telemetry.addData("0_velo", velo0);
            telemetry.addData("1_velo", velo1);
            telemetry.addData("2_velo", velo2);
            telemetry.addData("3_velo", velo3);

            telemetry.addData("hubInputVoltage", inputVoltage);
            telemetry.update();
        }
    }
}
