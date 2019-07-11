package us.ihmc.atlas.atlasStepUp;

import org.junit.jupiter.api.*;
import us.ihmc.atlas.*;
import us.ihmc.avatar.drcRobot.*;
import us.ihmc.avatar.stepUptest.*;
import us.ihmc.robotics.robotSide.*;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.*;
import us.ihmc.wholeBodyController.*;

import javax.annotation.*;

public class AtlasStepUp extends AvatarStepUp
{
   //FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3 , true, true);
   private final AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);//, simulationContactPoints);

   @Override
   @Test
   public void stepUpSmall() throws SimulationExceededMaximumTimeException
   {
      super.stepUpSmall();
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
