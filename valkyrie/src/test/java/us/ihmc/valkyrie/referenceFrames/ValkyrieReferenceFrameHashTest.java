package us.ihmc.valkyrie.referenceFrames;

import java.lang.reflect.InvocationTargetException;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.referenceFrames.ReferenceFrameHashTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieReferenceFrameHashTest extends ReferenceFrameHashTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, true);
   }

   @Override
   @Test// timeout = 30000, expected = IllegalArgumentException.class
   public void testAddingTwoFramesWithTheSameNameThrowsException()
   {
      super.testAddingTwoFramesWithTheSameNameThrowsException();
   }

   @Override
   @Test// timeout = 30000
   public void testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromFullRobotModelMethodsAreInTheHashList();
   }

   @Override
   @Test// timeout = 30000
   public void testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList()
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      super.testAllFramesGottenFromHumanoidReferenceFrameMethodsAreInTheHashList();
   }

   @Override
   @Test// timeout = 30000
   public void testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode()
   {
      super.testAllFramesInFullRobotModelMatchHumanoidReferenceFramesThroughHashCode();
   }

   @Override
   @Test// timeout = 30000
   public void testGetReferenceFrameFromHashCodeReturnsSameNamedFrames()
   {
      super.testGetReferenceFrameFromHashCodeReturnsSameNamedFrames();
   }
}
