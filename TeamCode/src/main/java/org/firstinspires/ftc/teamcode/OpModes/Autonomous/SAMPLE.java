package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Subsystems.VerticalLift;


public class SAMPLE extends LinearOpMode {
    @Override
    public void runOpMode() {
        //instantiate subsystems
        Pose2d beginPose = new Pose2d(11.7, 58.3, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VerticalLift lift = new VerticalLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);

    //initialize trajectories
        //default path to return to basket
        TrajectoryActionBuilder score = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(49, 52), Math.toRadians(45));

        //sample 1 grab
        TrajectoryActionBuilder grab1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(49, 46), Math.toRadians(-90));

        //sample 2 grab
        TrajectoryActionBuilder grab2 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(49, 46), Math.toRadians(-60));

        //sample 3 grab
        TrajectoryActionBuilder grab3 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(49, 46), Math.toRadians(-45));

        //park next to sub for level 1 ascent
        TrajectoryActionBuilder park = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(-90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(23, 11, Math.toRadians(180)), Math.toRadians(180));


        //don't forget to run grab() action during init to maintain possession of sample

        waitForStart();

        if (isStopRequested()) return;

        //Code
        Actions.runBlocking(score.build());

    }
}
