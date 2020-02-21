package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@Autonomous(name="GetFoundationAndParkCloseToWallBlue", group="Autonomous")

public class GetFoundationAndParkCloseToWallBlue extends LinearOpMode{
    DriveTrain robot = new DriveTrain();
    Intake intake = new Intake();
    FoundationMechanism foundation = new FoundationMechanism();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); //x: 48 30 y: 54 54. start facing backwards
        intake.init(hardwareMap);
        foundation.init(hardwareMap);

        waitForStart();

        robot.mecanumDriveStraightAlongZero(-0.5,-30,0); //x: 48 30 y: -24 -24
        robot.mecanumStrafe(0.5, 10,MovementDirection.RIGHT, 0); //58 40 y: -24 -24
        foundation.clampFoundation();
        robot.mecanumStrafe(0.5, 10,MovementDirection.LEFT, 0); //48 30 y: -24 -24
        robot.mecanumDriveStraightAlongZero(0.5,25,0); //x: 48 30 y: -49 -49
        robot.turnRobot(0.4, 90); //turn 90 counter-clockwise
        robot.mecanumDriveStraightAlongZero(-0.8,-25,90);
        foundation.releaseFoundation();
        robot.mecanumStrafe(0.5, 20,MovementDirection.RIGHT, 90); //48 30 y: -24 -24
        robot.mecanumDriveStraightAlongZero(0.5,40,90);
    }
}