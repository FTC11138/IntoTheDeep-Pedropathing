package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SpecimenSubsystem;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;
import org.firstinspires.ftc.teamcode.vision.SampleAlignmentProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Robot extends MecanumDrive {

    public Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public WebcamName webcam;
    public SampleAlignmentProcessor sampleAlignmentProcessor;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;


    public Limelight3A limelight;

    public RobotData data = new RobotData();

    public DepositSubsystem depositSubsystem;
    public IntakeSubsystem intakeSubsystem;
    public SpecimenSubsystem specimenSubsystem;
    public ArrayList<RE_SubsystemBase> subsystems;


    private static Robot instance = null;

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry_) {
        initializePedro(hardwareMap);

        CommandScheduler.getInstance().reset();

        telemetry = telemetry_;

        subsystems = new ArrayList<>();

        this.depositSubsystem = new DepositSubsystem(hardwareMap, names.lift, names.lift2, names.bucket);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, names.extension, names.arm1, names.arm2, names.intake, names.leds, names.intakeSensor);
        this.specimenSubsystem = new SpecimenSubsystem(hardwareMap, names.specimenClaw, names.specimenLift);

//        this.webcam = hardwareMap.get(WebcamName.class, names.webcam);
//        setupCamera();
//        stopCamera();



//        this.limelight = hardwareMap.get(Limelight3A.class, names.limelight);

    }

    public void write() {
        this.data.write(telemetry);
    }

    public void periodic() {
        for (RE_SubsystemBase subsystem : subsystems) {
            subsystem.periodic();
        }
    }

    public void updateData() {
        for (RE_SubsystemBase subsystem : subsystems) {
            subsystem.updateData();
        }
        this.data.currentPose = this.getPose();
    }

    private void setupCamera() {
        sampleAlignmentProcessor = new SampleAlignmentProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        aprilTagProcessor.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .addProcessor(sampleAlignmentProcessor)
                .setCamera(webcam)
                .build();
    }

    public void startCamera() {
        visionPortal.resumeStreaming();
    }

    public void stopCamera() {
        visionPortal.stopStreaming();
    }

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        return instance;
    }

}
