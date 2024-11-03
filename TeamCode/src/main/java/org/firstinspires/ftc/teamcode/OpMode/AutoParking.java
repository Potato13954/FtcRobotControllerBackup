package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "IntakeServoTest", group = "Autonomous")
public class AutoParking extends LinearOpMode {

    double power = 0.5;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private CRServo intake;
    private DcMotorEx armMotor;

    @Override
    public void runOpMode() {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(CRServo.class, "intake");
        armMotor = hardwareMap.get(DcMotorEx.class, "left_arm");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to brake when power is zero
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        // Unpark: Move forward to unpark
        IntakeRotation(intakeOperations.INTAKE_IN, 0.5f, 1);
        IntakeRotation(intakeOperations.INTAKE_OUT, 0.5f, 1);

        ArmRotation(armOperations.ARM_BACKWARDS, 1, 1);


        sleep(1000); // Pause before parking

        // Park: Move backward to park
        /*moveBackward(0.5, 1000); // Adjust power and time as needed
        stopMotors();*/
    }

    enum movement {
        FORWARD,
        BACKWARDS,
        LEFT,
        RIGHT
    }

    enum intakeOperations {
        INTAKE_IN,
        INTAKE_OUT
    }

    enum armOperations {
        ARM_FORWARD,
        ARM_BACKWARDS
    }

    private void move(movement movement, double power, int duration) {
        if(movement == AutoParking.movement.FORWARD)  {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(power);

        } else if(movement == AutoParking.movement.BACKWARDS) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoParking.movement.LEFT) {
            leftFrontDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightBackDrive.setPower(-power);

        } else if(movement == AutoParking.movement.RIGHT) {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightBackDrive.setPower(power);
        }
        sleep(duration);
        stopMotors();
    }

    private void IntakeRotation(intakeOperations operation, double power, int duration) {
        if(operation == intakeOperations.INTAKE_IN) {
            intake.setPower(power);
        } else if(operation == intakeOperations.INTAKE_OUT) {
            intake.setPower(-power);
        }
        sleep(duration);
        stopIntake();
    }
    private void ArmRotation(armOperations operation, double power, int duration) {
        if(operation == armOperations.ARM_FORWARD) {
            armMotor.setPower(power);
        } else if(operation == armOperations.ARM_BACKWARDS) {
            armMotor.setPower(-power);
        }
        sleep(duration);
        stopArm();
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void stopIntake() {
        intake.setPower(0);
    }
    private void stopArm() {
        armMotor.setPower(0);
    }
}