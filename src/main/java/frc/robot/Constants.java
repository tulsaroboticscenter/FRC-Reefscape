package frc.robot;


public final class Constants {

  public final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public final class Pneumatics {
    /** PSI - The pressure the compressor's sensor must read before it turns on */
    public static final int kCompressorActivationPressure = 70;
    /** PSI - The pressure the compressor's sensor must read before it turns off */
    public static final int kCompressorDeactivationPressure = 120;
  }

}
