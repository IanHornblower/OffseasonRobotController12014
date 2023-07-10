package org.firstinspires.ftc.teamcode.FSM.actions;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.FSM.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;
import org.firstinspires.ftc.teamcode.util.PID.SquareRootController;

//@Config
public class ToConeStack extends FiniteStateMachine {
    private final Robot robot;
    private final double targetAngle;
    private final SensorArmy.Color trackingColor;

    private final PIDController heading = new PIDController(-0.8,0.5,0.0);
    private final PIDController line = new PIDController(-0.045,0.002,0.008);
    private final SquareRootController wall = new SquareRootController(0.022,0.18,0);

    private double y_reloc = 0;
    private double x_reloc = 0;

    public static double wallDistance = 1.8; // in IN

    public ToConeStack(Robot robot, SensorArmy.Color trackingColor, double targetAngle) {
        this.robot = robot;
        this.trackingColor = trackingColor;
        this.targetAngle = targetAngle;

        addState("IDLE");
        addState("STARTING");
        addState("TO_WALl");
        addState("RELOCALIZE");
        addState("END");
    }

    public Pose2d getReloclization() {
        return new Pose2d(x_reloc, y_reloc, targetAngle);
    }

    @Override
    public void update() {
        switch(getState()) {
            case "IDLE":
                break;
            case "STARTING":
                robot.sensorArmy.setFollowingColor(trackingColor);

                line.setSetPoint(0);
                heading.setSetPoint(0);
                wall.setSetPoint(wallDistance);
                nextState();
                break;
            case "TO_WALl":
                double x = -wall.calculate(robot.sensorArmy.getDistanceToWall()); // redo -> sqrt control
                double y = line.calculate(robot.sensorArmy.getPosition());
                double h = -heading.calculate(AngleUnit.normalizeRadians(targetAngle - robot.drive.getPoseEstimate().getHeading()));

                if(robot.sensorArmy.getDistanceToWall() < wallDistance) {
                    x = 0.0;
                }

                robot.drive.setDrivePower(new Pose2d(x, y, 0));

                nextState(robot.sensorArmy.getDistanceToWall() < wallDistance);
                break;
            case "RELOCALIZE":
                // do
                double offset = -0.32 * robot.sensorArmy.getPosition();
                y_reloc = -12 - offset; // give in inches offset from center

                //x_reloc = -72 + robot.sensorArmy.getDistanceToWall();
                x_reloc = -62;

                robot.drive.setPoseEstimate(new Pose2d(x_reloc, y_reloc, robot.drive.getPoseEstimate().getHeading()));

                nextState();
                break;
            case "END":
                robot.drive.setDrivePower(new Pose2d(0, 0, 0));
                reset();
                break;
        }

    }
}
