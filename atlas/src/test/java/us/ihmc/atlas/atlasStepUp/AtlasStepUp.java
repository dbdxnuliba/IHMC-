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
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      super.stepUpSmall();
   }


   @Override
   @Test
   public void stepUpBig() throws SimulationExceededMaximumTimeException
   {
      super.stepUpBig();
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
/*
   public static void main(String[] args) throws Exception
   {
      AtlasStepUp test = new AtlasStepUp();
      test.stepUp();
   } */
}
