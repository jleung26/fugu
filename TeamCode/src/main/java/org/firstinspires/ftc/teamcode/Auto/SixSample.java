package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.BezierPoint;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystem.ExtendingOuttake;
import org.firstinspires.ftc.teamcode.Subsystem.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.Subsystem.VerticalSlides;
import org.firstinspires.ftc.teamcode.Util.RobotHardware;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "6 Sample Final", group = "A", preselectTeleOp = "Full TeleOp FINAL")
public class SixSample extends OpMode {
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
    private int samplesGrabbedFromSub = 0;

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
    private final Pose startPose = new Pose(6.25, 115, Math.toRadians(270));
    // X: bot against wall
    // Y: along closest edge of tile to bucket, just before covering the foam connecting teeth
    // back of bot towards bucket

    /** Bucket Scoring Pose */
    private final Pose scorePose = new Pose(16, 131, Math.toRadians(315)); // TODO: tuned, but can make more optimal

    /** First Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(19, 128, Math.toRadians(0)); // TODO: tuned, but can make more optimal
    // TODO: (e.g. with less movement from score, and turning instead) if need to save some time
    // old, very consistent pose, switch back if unable to tune new pos: 15, 128, Math.toRadians(0)

    /** Second Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(18, 132, Math.toRadians(0)); // tuned

    /** Third Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(27, 122, Math.toRadians(55)); // tuned

    private final Pose scoreControlPose = new Pose(68, 110, Math.toRadians(999)/* heading unused*/); // done

    private final Pose sub1Pose = new Pose(62, 98, Math.toRadians(270));

    private final Pose sub2Pose = new Pose(67, 98, Math.toRadians(270));

    private final Pose sub3Pose = new Pose(72, 98, Math.toRadians(270));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(62, 98, Math.toRadians(90)); // tuned

    private final Pose parkControlPose = new Pose(68, 110, Math.toRadians(999)/* heading unused*/); // done

    private final double SUB_GRAB_TIMEOUT = 1.5;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scoreToSub1, subStrafe, scoreFrom1, sub1ToSub2, scoreToSub2, scoreFrom2, sub2ToSub3, scoreToSub3, scoreFrom3, park;


    public void buildPaths() {

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. Straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setConstantHeadingInterpolation(pickup1Pose.getHeading())
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                // TODO: maybe constant heading here?
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

        scoreToSub1 = follower.pathBuilder() // to sub for first grab
                .addPath(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(sub1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sub1Pose.getHeading())
                .build();

        subStrafe = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub1Pose), new Point(sub3Pose)))
                .setConstantHeadingInterpolation(sub1Pose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .setZeroPowerAccelerationMultiplier(3)
                .addPath(new BezierLine(new Point(sub3Pose), new Point(sub1Pose)))
                .setConstantHeadingInterpolation(sub1Pose.getHeading())
                .build();

        scoreFrom1 = follower.pathBuilder() // from sub1 pose to scoring
                .addPath(new BezierCurve(new Point(sub1Pose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(sub1Pose.getHeading(), scorePose.getHeading())
                .build();

        sub1ToSub2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub1Pose), new Point(sub2Pose)))
                .setConstantHeadingInterpolation(sub2Pose.getHeading())
                .build();

        scoreToSub2 = follower.pathBuilder() // to sub for second grab
                .addPath(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(sub2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sub2Pose.getHeading())
                .build();

        scoreFrom2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub2Pose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(sub2Pose.getHeading(), scorePose.getHeading())
                .build();

        // backup, not for 7th sample
        sub2ToSub3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sub2Pose), new Point(sub3Pose)))
                .setConstantHeadingInterpolation(sub3Pose.getHeading())
                .build();

        scoreToSub3 = follower.pathBuilder() // to sub for first grab
                .addPath(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(sub3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sub3Pose.getHeading())
                .build();

        scoreFrom3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(sub3Pose), /* Control Point */ new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(sub3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .setZeroPowerAccelerationMultiplier(2)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                allToScoreAction();
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleAction();
                    prepToIntakeAction();
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= 0.6) { // delay
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(4);
                }
                break;
            case 4:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    // drive up to scoring
                    follower.followPath(scorePickup1,true);
                    setPathState(5);
                } else if (pathTimer.getElapsedTimeSeconds() > 2) { // backup if fail, intake's emptiness already assumed
                    setPathState(7);
                }
                break;
            case 5:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    transferAndPrepToScoreAction();
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()  && verticalSlides.getCurrentPos() > 800) {
                    depositSampleAction();
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    prepToIntakeAction();
                    follower.followPath(grabPickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(9);
                }
                break;
            case 9:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(10);
                }
                break;
            case 10:
                /* ready to transfer */
                if(/*arm stuff*/ horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    transferAndPrepToScoreAction();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleAction();
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    follower.followPath(grabPickup3,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    prepToIntakeAction();
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                    setPathState(14);
                }
                break;
            case 14:
                // wait until sample picked up
                if (intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    // retract and prep for transfer
                    retractIntakeAction();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3,true);
                    setPathState(15);
                }
                break;
            case 15:
                /* ready to transfer */
                if(/*arm stuff*/horizontalSlides.slidesRetracted && intake.wristFlippedUp) {
                    /* Transfer and prep to score*/
                    transferAndPrepToScoreAction();
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleAction();
                    setPathState(1001);
                }
                break;



            /// intaking from sub now, lots of convoluted paths
            /// INTAKE: bucket to sub 1, strafe to clear samples, then intake
            case 1001:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) { // delay
                    follower.followPath(scoreToSub1,true);
                    setPathState(1002);
                }
                break;
            case 1002:
                if (!follower.isBusy()) {
                    subIntakeAction();
                    setPathState(1003);
                }
                break;
            case 1003:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) { // back to sub1Pose now, should have cleared a decent row, so can try to intake
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendPartial()));
                    setPathState(1004); // sub 1 intaking (further down)
                }
                break;
            /// INTAKE: bucket to sub 2, then intake
            case 2001:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) { // delay
                    follower.followPath(scoreToSub2,true);
                    setPathState(2002);
                }
                break;
            case 2002:
                if (!follower.isBusy()) {
                    subIntakeAction();
                    setPathState(2003);
                }
                break;
            case 2003:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) { // back to sub1Pose now, should have cleared a decent row already, so can try to intake
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendPartial()));
                    setPathState(2004); // sub 2 intaking logic
                }
                break;
            /// INTAKE: bucket to sub 3, then intake
            case 3001:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) { // delay
                    follower.followPath(scoreToSub3,true);
                    setPathState(3002);
                }
                break;
            case 3002:
                if (!follower.isBusy()) {
                    subIntakeAction();
                    setPathState(3003);
                }
                break;
            case 3003:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) { // back to sub1Pose now, should have cleared a decent row, so can try to intake
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendPartial()));
                    setPathState(3004);
                }
                break;



            /// SCORE: sub 1 to bucket
            case 1005:
                if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom1, true);
                    setPathState(1006);
                }
            case 1006: // transfer from sub 1
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(1007);
                }
                break;
            case 1007: // score from sub 1
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleSubAction();
                    setPathState(2001); // back to sub to grab second sample
                }
                break;
            /// SCORE: sub 2 to bucket
            case 2006:
                if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom2, true);
                    setPathState(2007);
                }
            case 2007: // transfer from sub 2
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(2008);
                }
                break;
            case 2008: // score from sub 2
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleSubAction();
                    if (samplesGrabbedFromSub == 1) {
                        setPathState(3001); // not done, go back
                    } else if (samplesGrabbedFromSub == 2) {
                        setPathState(-1); // park, already done
                    } else {
                        telemetry.addLine("SAMPLES GRABBED INTEGER BROKE");
                        follower.turn(Math.toRadians(90), true);
                    }
                }
                break;
            /// SCORE: sub 3 to bucket
            case 3006:
                if (intake.wristFlippedUp) {
                    follower.followPath(scoreFrom3, true);
                    setPathState(3007);
                }
            case 3007: // transfer from sub 3
                if (horizontalSlides.slidesRetracted && intake.wristFlippedUp) { // ready to transfer, runs during middle of path
                    transferAndToScoreSubAction();
                    setPathState(3008);
                }
                break;
            case 3008: // score from sub 3
                if (!follower.isBusy() && verticalSlides.getCurrentPos() > 800) {
                    depositSampleSubAction();
                    setPathState(-1); // park
                }
                break;

            /// intaking logic at sub 1 pose
            case 1004:
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(1005); // sub 1 to bucket
                } else if (intake.chamberState == COLOR_TO_REJECT) { // reject sample and scoot to the side to grab another
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> horizontalSlides.extendBarely())
                    ));
                    follower.followPath(sub1ToSub2); // to sub 2 for another try
                    setPathState(2003); // go to last case of bucket to sub 2 logic
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendBarely())); // says extend, but actually retract
                    follower.followPath(sub1ToSub2);
                    setPathState(2003); // go to last case of bucket to sub 2 logic
                }
                break;

            /// intaking logic at sub 2 pose
            case 2004:
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(2006); // sub 2 to score bucket
                } else if (intake.chamberState == COLOR_TO_REJECT) { // reject sample and grab another
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> horizontalSlides.extendBarely())
                    ));
                    follower.followPath(sub2ToSub3);
                    setPathState(3003);
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT) {
                    runningActions.add(new InstantAction(() -> horizontalSlides.extendBarely()));
                    follower.followPath(sub2ToSub3);
                    setPathState(3003);
                }
                break;

            /// intaking logic at sub 3 pose
            case 3004:
                if (intake.chamberState != COLOR_TO_REJECT && intake.chamberState != Intake.IntakeChamberState.EMPTY) {
                    samplesGrabbedFromSub++;
                    retractIntakeAction();
                    setPathState(3006); // sub 3 to score bucket
                } else if (intake.chamberState == COLOR_TO_REJECT) { // reject sample and grab another
                    runningActions.add(new SequentialAction(
                            new InstantAction(() -> intake.flipUp()),
                            new SleepAction(0.2),
                            new InstantAction(() -> intake.reverse()),
                            new SleepAction(0.4),
                            new InstantAction(() -> intake.intake()),
                            new InstantAction(() -> intake.dropDown()),
                            new InstantAction(() -> horizontalSlides.extend()) // last ditch effort, might ruin sub
                    ));
                    setPathState(3004); // sets to same case to reset timer, and continue last ditch effort
                }
                else if (pathTimer.getElapsedTimeSeconds() > SUB_GRAB_TIMEOUT) { // nothing in reach, take too long, extend all the way; last ditch effort
                    runningActions.add(new InstantAction(() -> horizontalSlides.extend()));
                } else if (opmodeTimer.getElapsedTimeSeconds() > 29.5) {
                    retractIntakeAction();
                    follower.holdPoint(new Point(parkPose), parkPose.getHeading());
                }
                break;
            case -1: // park
                follower.followPath(park);
                stowToParkAction();
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
        outtake.closeClawLoose();
        outtake.toStow();
        intake.flipUp();
    }

    @Override
    public void init_loop() {
        // very important to select for when eventually >4 sample
        if (gamepad1.b || gamepad2.b) {
            redAlliance = true;
        } else if (gamepad1.x || gamepad2.x) {
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
        // action and bulk caching loops
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
        telemetry.addData("path timer seconds", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("current vert pos",  verticalSlides.getCurrentPos());

        telemetry.addLine("\n Pose");
        telemetry.addData("x: ", follower.getPose().getX());
        telemetry.addData("y: ", follower.getPose().getY());
        telemetry.addData("heading: ", follower.getPose().getHeading());

        telemetry.addData("\n Loop Times: ", elapsedtime.milliseconds());
        elapsedtime.reset();
    }

    // Abstracted Action Methods
    public void allToScoreAction() {
        // prep to score
        runningActions.add(new SequentialAction(
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.3),
                new InstantAction(() -> outtake.toVert()),
                new InstantAction(() -> outtake.armExtend.extendToScoreBucket()),
                new SleepAction(0.5), // TODO: tune this based on extension speed 0.3+0.5, probably only change 0.5
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void transferAndPrepToScoreAction() {
        // full transfer and prep to score sequence
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.setIntake(0.5)), // push sample all the way in, kinda jank, maybe not necessary
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.1), // TODO: play around with timings
                new InstantAction(() -> outtake.closeClawTight()),
                new SleepAction(0.45),
                new InstantAction(() -> outtake.toStow()),
                new InstantAction(() -> intake.setIntake(0)),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toVert())
        ));
    }

    public void transferAndToScoreSubAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> intake.setIntake(0.5)), // push sample all the way in, kinda jank, maybe not necessary
                new InstantAction(() -> outtake.toTransfer()),
                new SleepAction(0.1), // TODO: play around with timings
                new InstantAction(() -> outtake.closeClawTight()),
                new SleepAction(0.35),
                new InstantAction(() -> outtake.toStow()),
                new InstantAction(() -> intake.setIntake(0)),
                new InstantAction(() -> verticalSlides.raiseToHighBucket()),
                new SleepAction(0.2),
                new InstantAction(() -> outtake.toScoreBucket())
        ));
    }

    public void depositSampleAction() {
        // deposit in bucket, then retract all
        runningActions.add(new SequentialAction(
                new InstantAction(() -> outtake.toScoreBucket()),
                new SleepAction(0.3), // TODO: tune this based on extension speed 0.3+0.5, probably only to lower 0.5
                new InstantAction(() -> outtake.openClaw()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> outtake.toStow()),
                        new InstantAction(() -> verticalSlides.retract())
                )
        ));
    }

    public void depositSampleSubAction() {
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

    public void subIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.extendBarely()),
                new InstantAction(() -> intake.dropDown()),
                new InstantAction(() -> intake.intake())
        ));
    }

    public void retractIntakeAction() {
        runningActions.add(new SequentialAction(
                new InstantAction(() -> horizontalSlides.retract()),
                new InstantAction(() -> intake.flipUp()),
                new InstantAction(() -> intake.idle())
        ));
    }

    public void stowToParkAction() {
        runningActions.add(new SequentialAction(
                new ParallelAction(
                        new InstantAction(() -> intake.flipUp()),
                        new InstantAction(() -> intake.idle()),
                        new InstantAction(()-> outtake.toSubPark()),
                        new InstantAction(()-> outtake.openClaw())
                ),
                new SleepAction(0.7),
                new ParallelAction(
                        new InstantAction(() -> verticalSlides.retract()),
                        new InstantAction(()-> horizontalSlides.retract())
                )
        ));
    }

}
