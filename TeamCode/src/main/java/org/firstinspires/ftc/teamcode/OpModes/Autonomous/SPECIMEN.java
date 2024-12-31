package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;

@Autonomous
public class SPECIMEN extends LinearOpMode {
    @Override
    public void runOpMode() {
        //instantiate subsystems
        Pose2d beginPose = new Pose2d(-11.7, 58.3, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        //initialize trajectories
        TrajectoryActionBuilder path1 = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-7.7,35));

        TrajectoryActionBuilder path2 = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37.5, 35.5), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35.5, 0), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-56, 0), Math.toRadians(90))
                .lineToY(50)
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, 0), Math.toRadians(90))
                .lineToY(55);


        //don't forget to run grab() action during init to maintain possession of specimen

        waitForStart();

        if (isStopRequested()) return;

        //Code
        Actions.runBlocking(
            new SequentialAction(
        new ParallelAction(
                    path1.build(), //move to chamber
                    lift.raise()
                ),
                new ParallelAction(
            lift.lower(),
                    new SequentialAction(
                        new SleepAction(0),
                        claw.drop(lift.leftLift)
                    )
                ),
                path2.build()
            )
        );

    }
}

/*
.strafeTo(new Vector2d(-7.7,35))
        .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37.5, 35.5), Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-35.5, 0), Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(90))
        .lineToY(50)
                .setTangent(Math.toRadians(90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-45, 0), Math.toRadians(-90))
        .setTangent(Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-56, 0), Math.toRadians(90))
        .lineToY(50)
                .setTangent(Math.toRadians(-90))
        .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, 0), Math.toRadians(90))
        .lineToY(55)
        */

/*.strafeTo(new Vector2d(-7.7,35))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37.5, 35.5), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-34, 0), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-44, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-45, 50), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-50, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-55, 50), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-60, 0), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-61, 55), Math.toRadians(90))*/