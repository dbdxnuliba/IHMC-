package us.ihmc.robotics.math.trajectories.providers;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class YoSE3ConfigurationProviderTest
{
   private String name = "nameTest";

   private ReferenceFrame referenceFrame;
   private YoVariableRegistry registry;

   private YoSE3ConfigurationProvider provider;

   @BeforeEach
   public void setUp()
   {
      referenceFrame = ReferenceFrame.constructARootFrame("rootNameTEST");
      registry = new YoVariableRegistry("registryTEST");
   }

   @AfterEach
   public void tearDown()
   {
      referenceFrame = null;
      registry = null;
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test// timeout = 30000
   public void testConstructor()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, null);
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
   }

	@Test// timeout = 30000
   public void testGet()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FrameQuaternion orientationToPack = new FrameQuaternion();
      provider.getOrientation(orientationToPack);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FramePoint3D framePointToPack = new FramePoint3D();
      provider.getPosition(framePointToPack);

      assertEquals(referenceFrame, framePointToPack.getReferenceFrame());
   }

	@Test// timeout = 30000
   public void testSetPose()
   {
      provider = new YoSE3ConfigurationProvider(name, referenceFrame, registry);
      FramePose3D framePose;
      try
      {
         framePose = new FramePose3D();
         provider.setPose(framePose);
         fail();
      }
      catch (RuntimeException rte)
      {
      }

      framePose = new FramePose3D(referenceFrame);

      provider.setPose(framePose);
   }
}
