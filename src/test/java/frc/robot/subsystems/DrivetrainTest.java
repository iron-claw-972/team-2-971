// package frc.robot.subsystems;

// import org.junit.Before;
// import org.junit.Test;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// import static org.mockito.ArgumentMatchers.anyDouble;
// import static org.mockito.Mockito.mock;
// import static org.mockito.Mockito.times;
// import static org.mockito.Mockito.verify;
// import static org.mockito.Mockito.when;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.kauailabs.navx.frc.AHRS;

// public class DrivetrainTest {
//   private WPI_TalonFX m_mockLeftMotor1;
//   private WPI_TalonFX m_mockRightMotor1;
//   private AHRS m_mockGyro;
//   private DifferentialDrive m_mockDDrive;
//   private Drivetrain m_drive;

//   @Before
//   public void setup() {
//     m_mockLeftMotor1 = mock(WPI_TalonFX.class);
//     m_mockRightMotor1 = mock(WPI_TalonFX.class);

//     m_mockGyro = mock(AHRS.class);
//     when(m_mockGyro.getRotation2d()).thenReturn(new Rotation2d());

//     m_mockDDrive = mock(DifferentialDrive.class);
//   }

//   @Test
//   public void testArcadeDrive() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1);

//     m_drive.arcadeDrive(1, 1);

//     verify(m_mockLeftMotor1, times(1)).set(anyDouble());
//     verify(m_mockRightMotor1, times(1)).set(anyDouble());
//   }

//   @Test
//   public void testFFDrive() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1, m_mockDDrive);

//     m_drive.feedForwardDrive(1, 1);

//     verify(m_mockLeftMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockRightMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockDDrive, times(1)).feed();
//   }

//   @Test
//   public void testResetGyro() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1, m_mockGyro);

//     m_drive.resetGyro();

//     verify(m_mockGyro, times(2)).reset();
//   }

//   @Test
//   public void testResetEncoders() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1);

//     m_drive.resetEncoders();

//     verify(m_mockLeftMotor1, times(2)).setSelectedSensorPosition(0);
//     verify(m_mockRightMotor1, times(2)).setSelectedSensorPosition(0);
//   }

//   @Test
//   public void testTankDriveVolts() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1, m_mockDDrive);

//     m_drive.tankDriveVolts(1, 1);

//     verify(m_mockLeftMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockRightMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockDDrive, times(1)).feed();
//   }

//   @Test
//   public void testSetSpeeds() {
//     m_drive = new Drivetrain(m_mockLeftMotor1, m_mockRightMotor1, m_mockDDrive);

//     m_drive.setSpeeds(new DifferentialDriveWheelSpeeds());

//     verify(m_mockLeftMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockRightMotor1, times(1)).setVoltage(anyDouble());
//     verify(m_mockDDrive, times(1)).feed();
//   }
// }