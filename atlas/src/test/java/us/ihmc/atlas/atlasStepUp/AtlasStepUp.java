package us.ihmc.atlas.atlasStepUp;

import org.junit.jupiter.api.*;
import us.ihmc.atlas.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.stepUptest.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;

import javax.annotation.*;

public class AtlasStepUp extends AvatarStepUp
{
   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @Test
   public void stepUp() throws SimulationExceededMaximumTimeException
   {
      super.stepUp();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return atlasRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return "Atlas";
   }

   public static void main(String[] args) throws Exception
   {
      AvatarStepUp test = new AvatarStepUp();
      test.stepUp();
   }
}
