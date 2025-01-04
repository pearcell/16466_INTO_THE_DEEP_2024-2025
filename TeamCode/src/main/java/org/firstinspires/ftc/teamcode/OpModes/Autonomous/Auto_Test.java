package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        }
    }
}
