package org.firstinspires.ftc.teamcode.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.DriveConstants;

public class RobotData {

    public long loopTime = System.currentTimeMillis();

    public int liftPosition1 = 0;
    public int liftPosition2 = 0;
    public double bucketPosition = 0;
    public DepositSubsystem.BucketState bucketState = DepositSubsystem.BucketState.NONE;

    public int extensionPosition = 0;
    public int extensionTarget = 0;
    public double armPosition1 = 0;
    public double armPosition2 = 0;
    public IntakeSubsystem.ArmState armState = IntakeSubsystem.ArmState.NONE;
    public IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.IntakeState.STOP;
    public double intakeDistance = 0;

    public int specimenLiftPosition = 0;
    public SpecimenSubsystem.SpecimenClawState specimenClawState = SpecimenSubsystem.SpecimenClawState.OPEN;
    public SpecimenSubsystem.SpecimenLiftState specimenLiftState = SpecimenSubsystem.SpecimenLiftState.DOWN;

    public Pose currentPose = new Pose(0,0, Math.toRadians(0));

    public boolean scoring = false;
    public boolean sampleLoaded = false;
    public boolean intaking = false;

    public void write(Telemetry telemetry) {

        telemetry.addData("LOOP TIME", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

        telemetry.addLine();

        telemetry.addData("POSE", this.currentPose);
        telemetry.addData("PATH TIMEOUT CONSTRAINT", DriveConstants.pathEndTimeoutConstraint);
        telemetry.addData("BUSY", Robot.getInstance().isBusy());

        telemetry.addLine();

        telemetry.addData("SCORING", this.scoring);
        telemetry.addData("INTAKING", this.intaking);
        telemetry.addData("SAMPLE LOADED", this.sampleLoaded);

        telemetry.addLine();

        telemetry.addData("Lift Position 1", this.liftPosition1);
        telemetry.addData("Lift Position 2", this.liftPosition2);
        telemetry.addData("Wrist Position", this.bucketPosition);
        telemetry.addData("Bucket State", this.bucketState);

        telemetry.addLine();

        telemetry.addData("Extension Position", this.extensionPosition);
        telemetry.addData("Extension Target", this.extensionTarget);
        telemetry.addData("Arm Position 1", this.armPosition1);
        telemetry.addData("Arm Position 2", this.armPosition2);
        telemetry.addData("Arm State", this.armState);
        telemetry.addData("Intake State", this.intakeState);
        telemetry.addData("Intake Distance", this.intakeDistance);

        telemetry.addLine();

        telemetry.addData("Specimen Lift Position", this.specimenLiftPosition);
        telemetry.addData("Specimen Claw State", this.specimenClawState);
        telemetry.addData("Specimen Lift State", this.specimenLiftState);

        telemetry.addLine();

        telemetry.update();
    }

    public void startScoring() {
        scoring = true;
        intaking = false;
    }

    public void stopScoring() {
        scoring = false;
        intaking = false;
    }

    public void startIntaking() {
        scoring = false;
        intaking = true;
    }

    public void stopIntaking() {
        scoring = false;
        intaking = false;
    }

    public void setSampleLoaded() {
        sampleLoaded = true;
    }

    public void setSampleUnloaded() {
        sampleLoaded = false;
    }

}
