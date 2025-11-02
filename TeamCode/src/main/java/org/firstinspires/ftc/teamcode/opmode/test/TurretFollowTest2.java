package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterIntake;
import org.firstinspires.ftc.teamcode.robot.subsystems.BetterShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.misc.SOTM2;
import org.firstinspires.ftc.teamcode.util.misc.VoltageCompFollower;
import com.pedropathing.pathgen.Vector;
@Config
@TeleOp(name="turret follow test + shooter tuner with kv")
public class TurretFollowTest2 extends OpMode {
    private Turret turret;
    public static double shooterSpeed;
    public static double shooterPitch;
    public static double kV = 0;
    public static double kP = 0.003;
    public static double kD = 0.00003;
    private double lastTimeStamp = 0;
    private double lastAngleToGoal;
    private Follower follower;
    private BetterIntake intake;
    private SOTM2 sotm2;
    private BetterShooter shooter;
    private final Pose startPose = new Pose(54, 6, Math.toRadians(90));
    // i don't think angle matters here in the sotm calculation
    private final Pose goalPose = new Pose(0, 144, Math.toRadians(45));
    @Override
    public void loop() {
        turret.setPDCoefficients(kP, kD);
        shooter.setTargetVelocity(shooterSpeed);
        shooter.setShooterPitch(Math.toRadians(shooterPitch));
        double[] target = sotm2.calculateAzimuthThetaVelocity(follower.getPose(), new Vector());
        // set azimuth
        turret.setTarget(target[0]);
        System.out.println(target[0]);

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        double period = currentTimeStamp - lastTimeStamp;

        double dx = goalPose.getX() - follower.getPose().getX();
        double dy = goalPose.getY() - follower.getPose().getY();
        double currentAngleToGoal = Math.atan2(-dx, dy) - follower.getPose().getHeading() + Math.toRadians(90);
        double vGoal = (currentAngleToGoal-lastAngleToGoal)/period;

        double ff = kV * vGoal;
        turret.setFeedforward(ff);

        lastAngleToGoal = currentAngleToGoal;
        lastTimeStamp = currentTimeStamp;
        follower.update();
        turret.update();
        shooter.update();
        intake.update();
        telemetry.addData("distance", Math.hypot(dx, dy));
        telemetry.addData("shooter speed", shooter.getCurrentVelocity());
        telemetry.addData("turret at target? 1.5% error", turret.atTarget(30));
        telemetry.update();

    }

    @Override
    public void init() {
        follower = new VoltageCompFollower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
        turret = new Turret(hardwareMap);
        turret.resetEncoder();
        shooter = new BetterShooter(hardwareMap);
        shooter.setShooterOn(true);
        intake = new BetterIntake(hardwareMap);
        intake.setIntakeOn(true);
        // intake.state = BetterIntake.IntakeState.INTAKE_FAST;
        intake.intakeDown();
        sotm2 = new SOTM2(goalPose);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double dx = goalPose.getX() - follower.getPose().getX();
        double dy = goalPose.getY() - follower.getPose().getY();
        lastAngleToGoal = Math.atan2(-dx, dy) - follower.getPose().getHeading() + Math.toRadians(90);
    }

    @Override
    public void start() {
        //follower.startTeleopDrive();
    }
}
