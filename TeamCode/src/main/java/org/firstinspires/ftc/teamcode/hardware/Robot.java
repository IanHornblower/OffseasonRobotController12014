package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxUnsupportedCommandException;
import com.qualcomm.hardware.lynx.commands.LynxCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Camera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Eyelids;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Faga;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

import org.firstinspires.ftc.teamcode.hardware.subsystems.SensorArmy;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.PID.PIDController;

public class Robot implements Subsystem {

    public Eyelids eyelids;
    public Faga faga;
    public Intake intake;
    public Lift lift;
    public SensorArmy sensorArmy;
    public SampleMecanumDrive drive;
    public Camera camera;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        eyelids = new Eyelids(hardwareMap);
        faga = new Faga(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
        sensorArmy = new SensorArmy(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new Camera(hardwareMap, telemetry);
    }

    public void init() throws InterruptedException {
        // does nothing atm
    }

    @Override
    public void periodic(){
        faga.periodic();
        lift.periodic();
        sensorArmy.periodic();
        //drive.update();

        grabConeLoop();
        returnConeLoop();
    }

    public void autoUpdate(){
        faga.periodic();
        lift.periodic();
        sensorArmy.periodic();

        grabConeLoop();
        returnConeLoop();
        toWallLoop();
    }

    public enum TO_WALL {
        IDLE,
        START,
        RUNNING,
        END,
    }

    public TO_WALL tw_state = TO_WALL.IDLE;

    public double forwardSpeed = 0.3;
    int cycle = 1;

    // TODO: USE bulk calls to get all lift and fourbar motor information, maybe for rr aswell

    PIDController heading = new PIDController(-0.8,0.05,0.0);
    PIDController line = new PIDController(0.04,0,0); // Maybe motion profile the line follow

    double startAngle = Math.toRadians(180);

    public void toWall(double angle) {
        tw_state = TO_WALL.START;
        startAngle = angle;
    }

    public void toWall() {
        tw_state = TO_WALL.START;
    }

    public static double wallDistance = 2.2;
    public static double downSpeed = -0.65;
    public static double timeoutDuration = 1.2;

    ElapsedTime wallTimer = new ElapsedTime();

    public void toWallLoop() {
        switch (tw_state) {
            case IDLE:
                break;
            case START:
                wallTimer.reset();
                if(sensorArmy.folowingColor.equals(SensorArmy.Color.RED)) {
                    line.setP(-0.065);
                }
                else {
                    line.setP(-0.045); // blue
                }

                line.setSetPoint(0);
                heading.setSetPoint(0);
                //startAngle = drive.getPoseEstimate().getHeading();

                tw_state = TO_WALL.RUNNING;
                break;
            case RUNNING:
                double x = forwardSpeed;
                double y = line.calculate(sensorArmy.getPosition());
                double h = heading.calculate(AngleUnit.normalizeRadians(startAngle - drive.getPoseEstimate().getHeading()));

                if(sensorArmy.getDistanceToWall() < wallDistance) {
                    x = 0.0;
                }

                drive.setDrivePower(new Pose2d(x, y, h));
                //if(sensorArmy.isOnLine() && sensorArmy.getDistanceToWall() < wallDistance) tw_state = TO_WALL.END;
                if(sensorArmy.getDistanceToWall() < wallDistance || wallTimer.seconds() > timeoutDuration) tw_state = TO_WALL.END;
                break;
            case END:
                drive.stop();

                double yR = -12+cycle/2.0;
                yR = -12.5;

                if(startAngle == 0.0) {
                    drive.setPoseEstimate(new Pose2d(
                            //-65+sensorArmy.getDistanceToWall(),
                            63.8,
                            yR,
                            Math.toRadians(0)));
                } else {
                    drive.setPoseEstimate(new Pose2d(
                            //-65+sensorArmy.getDistanceToWall(),
                            -63.8,
                            yR,
                            Math.toRadians(180)));
                }
                tw_state = TO_WALL.IDLE;
                break;
        }
    }


    public enum grabCone {
        IDLE,
        START,
        DROPPING,
        GRAB,
        PRIME,

    }


    ElapsedTime grabConeTimer = new ElapsedTime();
    public grabCone grabConeState = grabCone.IDLE;
    boolean auto = false;
    boolean beacon = false;
    boolean tele = false;

    public void grabCone() {
        grabConeState = grabCone.START;
        //faga.clawClose();
    }
    public void grabConeAuto() {
        grabConeState = grabCone.START;
        auto = true;
        //faga.clawClose();
    }

    public void grabConeTele() {
        grabConeState = grabCone.START;
        tele = true;
    }

    public void grabConeBeacon() {
        grabConeState = grabCone.START;
        beacon = true;
    }

    public void grabConeLoop() {
        switch (grabConeState) {
            case IDLE:
                break;
            case START:
                grabConeState = grabCone.DROPPING;
                grabConeTimer.reset();

                if(tele) {
                    lift.setPosition(0);
                }
                break;
            case DROPPING:
                faga.setFourbarPosition(faga.getFourbar().getCurrentPosition() - 600);
                if(grabConeTimer.seconds() > 0.15) {
                    grabConeState = grabCone.GRAB;
                    grabConeTimer.reset();
                }
                break;
            case GRAB:
                faga.clawClose();
                grabConeState = grabCone.PRIME;
                break;
            case PRIME:
                if(auto) faga.setArticulation(RobotConstants.Faga.Articulator.autoStack);
                if(!beacon && !tele) faga.setToPrime();
                if(tele) faga.setToPrimeTele();
                if(beacon) faga.setFourbarPosition(faga.getFourbar().getCurrentPosition() + 200);
                grabConeState = grabCone.IDLE;
                break;
        }
    }

    enum returnCone {
        IDLE,
        START,
        DROP,
        RETURNED,
        OPEN

    }

    ElapsedTime returnConeTimer = new ElapsedTime();
    returnCone returnConeState = returnCone.IDLE;

    double returnOffset = 0.0;
    double pos = 0;

    public void returnCone() {
        returnConeState = returnCone.START;
        faga.clawOpen();
        returnConeTimer.reset();
        pos = 0;
    }

    public void returnCone(double pos) {
        returnConeState = returnCone.START;
        faga.clawOpen();
        returnConeTimer.reset();
        this.pos = pos;
    }

    public void returnConeLoop() {
        switch (returnConeState) {
            case IDLE:
                break;
            case START:
                if(returnConeTimer.seconds() > returnOffset || returnOffset == 0.0) {
                    returnConeTimer.reset();
                    returnConeState = returnCone.DROP;
                }

                break;
            case DROP:
                faga.clawOpen();
                if(returnConeTimer.seconds() > 0.35) returnConeState = returnCone.RETURNED;
                break;
            case RETURNED:
                faga.clawTransfer();
                lift.setPosition(pos);
                faga.returnToIntake();
                if(faga.getFourbar().getCurrentPosition() < 2200) returnConeState = returnCone.OPEN;
                break;
            case OPEN:
                faga.clawOpen();
                returnConeState = returnCone.IDLE;
        }
    }
}
