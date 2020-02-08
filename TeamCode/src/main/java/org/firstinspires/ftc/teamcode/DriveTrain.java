package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class DriveTrain {
    public DcMotor  rightFrontMotor, leftFrontMotor, rightBackMotor, leftBackMotor = null;
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        rightFrontMotor  = hwMap.get(DcMotor.class, "rfm");
        leftFrontMotor  = hwMap.get(DcMotor.class, "lfm");
        rightBackMotor = hwMap.get(DcMotor.class, "rbm");
        leftBackMotor  = hwMap.get(DcMotor.class, "lbm");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);

        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void setTargetPositions(double inches, double COUNTS_PER_INCH) {
        int newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newRightBackTarget = rightBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        int newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightBackMotor.setTargetPosition(newLeftBackTarget);
        leftBackMotor.setTargetPosition(newLeftFrontTarget);
    }

    public void activateRunToPositionMode() {
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void activateRunUsingEncoderMode() {
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shutDown() {
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftBackMotor.setPower(0);
    }

    public void mecanumDriveStraightAlongZero(double speed, double inches, BNO055IMU imu, double COUNTS_PER_INCH) {
        setTargetPositions(inches, COUNTS_PER_INCH);
        activateRunToPositionMode();

        double leftSpeed = speed; double rightSpeed = speed;
        setPower(speed, speed, speed, speed);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double relativeAngle = inches > 0 ? 0 : 180;

        while ((rightFrontMotor.isBusy() && leftFrontMotor.isBusy())) {

            double error = getError(relativeAngle, imu);
            double steeringError = inches >= 0 ? getSteeringError(error, 0.01) : -1 * getSteeringError(error, 0.01);

            leftSpeed = speed-steeringError;
            rightSpeed = speed+steeringError;

            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

            if (max > 1) {
                leftSpeed /= (max);
                rightSpeed /= (max);
            }

            setPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
        }

        activateRunUsingEncoderMode();
        setPower(0,0,0,0);
    }

    public void mecanumStrafe(double speed, double inches, BNO055IMU imu, MovementDirection dir, double COUNTS_PER_INCH) {
        setTargetPositions(inches, COUNTS_PER_INCH);

        double topLeftSpeed = speed; double topRightSpeed = speed;
        double bottomLeftSpeed = speed; double bottomRightSpeed = speed;

        if (dir == MovementDirection.RIGHT) {
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (dir == MovementDirection.LEFT) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        activateRunToPositionMode();
        setPower(topLeftSpeed, topRightSpeed, bottomLeftSpeed, bottomRightSpeed);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double relativeAngle = dir == MovementDirection.LEFT ? 90 : 270;

        while ((rightFrontMotor.isBusy() && leftFrontMotor.isBusy())) {
            double error = getError(relativeAngle, imu);
            double steeringError = dir == MovementDirection.RIGHT ? getSteeringError(error, 0.01) : -1 * getSteeringError(error, 0.01);

            topLeftSpeed = topLeftSpeed-steeringError;
            topRightSpeed = topRightSpeed+steeringError;
            bottomLeftSpeed = bottomLeftSpeed-steeringError;
            bottomRightSpeed = bottomRightSpeed+steeringError;

            double max = Math.max(Math.abs(topLeftSpeed), Math.abs(topRightSpeed));

            if (max > 1) {
                topRightSpeed /= max;
                bottomRightSpeed /= max;
                topLeftSpeed /= max;
                bottomLeftSpeed /= max;
            }

            setPower(topLeftSpeed, topRightSpeed, bottomRightSpeed, bottomLeftSpeed);
        }

        if (dir == MovementDirection.RIGHT) {
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else if (dir == MovementDirection.LEFT) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        activateRunUsingEncoderMode();
        setPower(0,0,0,0);
    }

    public void turnRobot(double speed, double degrees, BNO055IMU imu) { //turns counterclockwise
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double target = (degrees + angles.firstAngle);
        double robotError = getError(target, imu);

        if (degrees < 0) {
            speed *= -1;
        }

        setPower(-speed, speed, -speed, speed);
        target = (target + 360) % 360;

        while (angles.firstAngle < target) {
            robotError = getError(target, imu);
            if (robotError < 10) {
                setPower(-speed/2, speed/2, -speed/2, speed/2);
            } else {
                setPower(-speed, speed, -speed, speed);
            }
        }
    }

    public double getError(double targetAngle, BNO055IMU imu) { // calculate error in -179 to +180 range
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  {
            robotError -= 360;
        }
        while (robotError <= -180) {
            robotError += 360;
        }
        return robotError;
    }

    public double getSteeringError(double error, double P_COEFF) {
        return error * P_COEFF;
    }

}
