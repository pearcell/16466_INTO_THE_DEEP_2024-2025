package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Tests;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@TeleOp
public class Auto_Test extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(15.5, 63, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        HorizontalExtension intake = new HorizontalExtension(hardwareMap);

        TrajectoryActionBuilder scorePreload = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(new Vector2d(57.4, 58.7), Math.toRadians(49.3)), Math.toRadians(45), new TranslationalVelConstraint(20));

        TrajectoryActionBuilder park = scorePreload.endTrajectory().fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(new Vector2d(38, 11), Math.toRadians(-90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(new Vector2d(25, 11), Math.toRadians(180)), Math.toRadians(180));

        telemetry.update();
        waitForStart();

        Actions.runBlocking(
                intake.retract()
        );

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        scorePreload.build(),
                        park.build()
                )

        );
    }
}
