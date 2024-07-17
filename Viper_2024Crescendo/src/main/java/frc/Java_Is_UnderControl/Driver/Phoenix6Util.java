package frc.Java_Is_UnderControl.Driver;

import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;

public class Phoenix6Util {
  /**
   * checks the specified error code for issues
   *
   * @param errorCode error code
   * @param message   message to print if error happens
   */
  public static void checkError(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      DriverStation.reportError(message + " " + errorCode, false);
    }
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
    StatusCode code = function.get();
    int tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
      code = function.get();
      tries++;
    }
    if (code != StatusCode.OK) {
      DriverStation.reportError("Failed to execute phoenix pro api call after " + numTries + " attempts", false);
      return false;
    }
    return true;
  }

  /**
   * checks the specified error code and throws an exception if there are any
   * issues
   *
   * @param errorCode error code
   * @param message   message to print if error happens
   */
  public static void checkErrorWithThrow(ErrorCode errorCode, String message) {
    if (errorCode != ErrorCode.OK) {
      throw new RuntimeException(message + " " + errorCode);
    }
  }

  public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
    return checkErrorAndRetry(function, 1);
  }

  public static void checkErrorAndRetry(Supplier<StatusCode> function) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'checkErrorAndRetry'");
  }
}
