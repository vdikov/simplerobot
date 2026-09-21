package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorWithMotionMagic extends SubsystemBase {
    private static String RIO_CAN_LOOP_NAME = "rio";
    private static int TOP_ELEVATOR_CAN_ID = 16;
    private static int BOTTOM_ELEVATOR_CAN_ID = 17;

    private final CANBus bus = new CANBus(RIO_CAN_LOOP_NAME);
    private final TalonFX topMotor = new TalonFX(TOP_ELEVATOR_CAN_ID, bus);
    private final TalonFX bottomMotor = new TalonFX(BOTTOM_ELEVATOR_CAN_ID, bus);

    // Simulation-only definitions
    private final TalonFXSimState topMotorSim = topMotor.getSimState();
    private final ElevatorSim elevatorSim;

    // Constants for the mechanism simulation parameters
    private static final double kGearReduction = 70.0 / 12.0; // 70:12 gearbox
    private static final Mass kCarriageMass = Kilograms.of(5.0); // 5 kg elevator carriage
    private static final Distance kSprocketPitchRadius = Inches.of(1.751 / 2.0); // 1" radius pulley
    private static final Distance kMinHeight = Meters.of(0.0);
    private static final Distance kMaxHeight = Meters.of(0.5);

    public ElevatorWithMotionMagic(NetworkTableInstance nt) {
        // Configure the motors.
        TalonFXConfiguration configs = new TalonFXConfiguration()
                .withMotorOutput(
                        new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(Amps.of(17)))
                .withSlot0(new Slot0Configs()
                        .withKS(0.0)
                        .withKG(0.0)
                        .withKV(0.0)
                        .withKA(0.0)
                        .withKP(0.0)
                        .withKI(0.0)
                        .withKD(0.0));
        topMotor.getConfigurator().apply(configs);
        bottomMotor.getConfigurator().apply(configs);

        bottomMotor.setControl(
                new Follower(TOP_ELEVATOR_CAN_ID, MotorAlignmentValue.Aligned));

        // 2. Initialize the WPILib elevator physics simulation plant
        elevatorSim = new ElevatorSim(
                DCMotor.getKrakenX60(2), // Modeling 2 motors total on the gearbox
                kGearReduction,
                kCarriageMass.in(Kilograms),
                kSprocketPitchRadius.in(Meters),
                kMinHeight.in(Meters),
                kMaxHeight.in(Meters),
                true, // Simulate gravity
                0.0 // Starting height
        );
    }

    /**
     * Convert a linear travel distance in meters to the equivalent number of motor
     * rotations, taking into account the sprocket radius and gear reduction.
     * 
     * Can be applied to both position (m) and velocity (m/s) conversions, as the
     * units are consistent.
     * 
     * @param linearTravelMeters
     * @return The equivalent number of motor rotations for the given linear travel
     *         distance.
     */
    private static double linearTravelToMotorRotations(double linearTravelMeters) {
        return (linearTravelMeters / (2 * Math.PI * kSprocketPitchRadius.in(Meters))) * kGearReduction;
    }

    @Override
    public void simulationPeriodic() {
        // Feed the simulated battery voltage into the motor controller simulation state
        topMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage()); //

        // Fetch the motor's requested output voltage from the code
        double motorVoltage = topMotorSim.getMotorVoltage(); //

        // Pass the voltage into the physical simulation environment
        elevatorSim.setInput(motorVoltage);

        // Step the simulation forward by 20ms (standard robot loop)
        elevatorSim.update(0.020);

        // Convert the linear height back to motor rotations to update internal sensor
        // data
        // Rotations = Linear distance / Circumference * Gear Reduction
        double linearPosition = elevatorSim.getPositionMeters();
        double linearVelocity = elevatorSim.getVelocityMetersPerSecond();

        // Force the simulated motor's sensor to read our simulated physical mechanism
        // values
        topMotorSim.setRawRotorPosition(linearTravelToMotorRotations(linearPosition));
        topMotorSim.setRotorVelocity(linearTravelToMotorRotations(linearVelocity));
    }
}
