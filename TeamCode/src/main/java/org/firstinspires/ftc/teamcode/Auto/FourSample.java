package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.Outtake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "4 Sample", group = "A")
public class FourSample extends OpMode {
    // declaring subsystems
    RobotHardware robotHardware = new RobotHardware();
    VerticalSlides verticalSlides = new VerticalSlides();
    HorizontalSlides horizontalSlides = new HorizontalSlides();
    ExtendingOuttake outtake = new ExtendingOuttake();
    Intake intake = new Intake();
    Follower follower;
    Timer pathTimer, actionTimer, opmodeTimer;

    // booleans
    // team color
    // spec/sample
    boolean redAlliance = true;
    Intake.IntakeChamberState COLOR_TO_REJECT;

    // Action stuff
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    // bulk cache reading
    private List<LynxModule> allHubs;

    // adds ability to graph telemetry values
    private MultipleTelemetry dashboardTelemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

    // loop time tracking
    private ElapsedTime elapsedtime;

    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
    // no visualizer access rn, so need to determine poses
    // also modify extendPartial fraction, also find 42" extension limit
    // use 42" limit in visualizer, might have to input 84" bot lol

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(6, 111, Math.toRadians(270));

    /** Bucket Scoring Pose */
    private final Pose scorePose = new Pose(14, 129, Math.toRadians(330));

    /** First Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(16, 125, Math.toRadians(330));

    /** Second Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(16, 130, Math.toRadians(340));

    /** Third Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(16, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(62, 98, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used as control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(62, 98, Math.toRadians(0));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;


    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. Straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                allToScoreAction();
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    // deposit sample
                    depositSampleAction();

                    // extend to prep for picking up sample
                    prepToIntakeAction();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);

                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    // TODO: there's a chance this is too fast. If so, we'll have to use something with slower or step-like extension

                    setPathState(3);
                }
                break;
            case 3:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    // drive up to scoring
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* ready to transfer */
                if(outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.STOW && horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    transferAndScoreAction();

                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    // deposit and move onto next sample
                    depositSampleAction();
                    prepToIntakeAction();

                    follower.followPath(grabPickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));

                    setPathState(7);
                }
                break;
            case 7:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* ready to transfer */
                if(outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.STOW && horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    transferAndScoreAction();

                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    // deposit and move onto next sample
                    depositSampleAction();
                    prepToIntakeAction();

                    follower.followPath(grabPickup3,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));

                    setPathState(11);
                }
                break;
            case 11:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3,true);
                    setPathState(12);
                }
                break;
            case 12:
                /* ready to transfer */
                if(outtake.armPitch.armPos == ExtendingOuttake.ArmPitch.STATE.STOW && horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    transferAndScoreAction();

                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    // deposit and move onto next sample
                    depositSampleAction();
                    prepToIntakeAction();

                    follower.followPath(park,true);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */
                    // TODO: find a pose where the arm won't be out of servo range after teleop start
                    //       or if we're feeling frisky, we try grabbing another sample to-go for teleop
                    //       if we're feeling crazy, we try a 5/6 sample auto

                    /* Set the state to a case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // init subsystems
        elapsedtime = new ElapsedTime();
        robotHardware.initialize(this);
        verticalSlides.initialize(this, robotHardware, true);
        horizontalSlides.initialize(this, robotHardware, true);
        outtake.initialize(this, robotHardware);
        intake.initialize(this, robotHardware);

        // bulk cache reading
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        buildPaths();

        // ready to go
        outtake.toStow();
        intake.flipUp();
    }

    @Override
    public void init_loop() {
        // very important to select for when eventually >4 sample or >5 spec
        if (gamepad1.a || gamepad2.a) {
            redAlliance = true;
        } else if (gamepad1.b || gamepad2.b) {
            redAlliance = false;
        }

        COLOR_TO_REJECT = (redAlliance ? Intake.IntakeChamberState.BLUE : Intake.IntakeChamberState.RED);
        telemetry.addData("red alliance? ", redAlliance);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // clearing bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for RR Action execution
        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            if (action.run(packet)) { // actually running actions
                newActions.add(action); // if failed (run() returns true), try again next loop
            }
        }
        runningActions = newActions;

        // loops
        verticalSlides.operate();
        horizontalSlides.operateAuto();
        intake.operateColorChecking();

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addLine("\n States");
        telemetry.addData("path state: ", pathState);
        telemetry.addData("isBusy? ", follower.isBusy());
        telemetry.addData("pinpoint cooked? ", follower.isLocalizationNAN());
        telemetry.addData("robot stuck? ", follower.isRobotStuck());

        telemetry.addLine("\n Pose");
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());

        telemetry.addData("\n Loop Times: ", elapsedtime.milliseconds());
    }

    // Abstracted Action Methods
    public void allToScoreAction() {
        // prep to score
        runningActions.add(new SequentialAction(
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> intake.flipUp()),
                new SleepAction(0.5),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void transferAndScoreAction() {
        // full transfer and prep to score sequence
        runningActions.add(new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> outtake.openClaw()),
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                ),
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.2), // TODO: reflect timings from teleop
                new InstantAction(() -> outtake.closeClawLoose()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toStow()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> intake.flipUp()),
                new SleepAction(0.5),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void depositSampleAction() {
        // deposit in bucket, then retract all
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void prepToIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendPartial()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> intake.intake())
        ));
    }

    public void retractIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.retract()),
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.neutral())
        ));
    }

    public void extendIntakeSlowlyAction() {
        double DELAY = 0.05; // seconds, not milliseconds
        // This is jank, but I don't wanna touch the PID loop that's running, so this will have to do for now
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend()),
                new SleepAction(DELAY),
                new InstantAction(() -> horizontalSlides.stepExtend())
        ));
    }
}
