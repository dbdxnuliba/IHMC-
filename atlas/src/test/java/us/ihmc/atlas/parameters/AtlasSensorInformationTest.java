package us.ihmc.atlas.parameters;

import static org.junit.Assert.assertFalse;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
public class AtlasSensorInformationTest
{
   @Test// timeout = 30000
   public void testSendRobotDataToROSIsFalse()
   {
      assertFalse("Do not check in SEND_ROBOT_DATA_TO_ROS = true!!", AtlasSensorInformation.SEND_ROBOT_DATA_TO_ROS);
   }
}
