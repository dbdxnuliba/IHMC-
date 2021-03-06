package us.ihmc.atlas.networkProcessor.modules.mocap;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import optiTrack.MocapMarker;
import optiTrack.MocapRigidBody;
import us.ihmc.avatar.networkProcessor.modules.mocap.MocapToPelvisFrameConverter;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.thread.ThreadTools;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.EXCLUDE)
public class MocapToPelvisFrameConverterTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private final Random random = new Random(456654321123L);

   @Before
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before: ");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      ReferenceFrameTools.clearWorldFrameTree();
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 10000)
   public void testForFrameConversionNoPelvisMotion()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      ReferenceFrame pelvisFrame = ReferenceFrame.constructFrameWithUnchangingTranslationFromParent("pelvisFrame", ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 0.0, 0.0));
      ReferenceFrame mocapFrame = ReferenceFrame.constructFrameWithUnchangingTranslationFromParent("mocapFrame", ReferenceFrame.getWorldFrame(), new Vector3D(0.0, 0.0, 1.0));
      MocapRigidBody markerRigidBody = createMocapRigidBody(pelvisFrame, mocapFrame);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(pelvisFrame, markerRigidBody);

      RigidBodyTransform computedPelvisToWorldTransform = new RigidBodyTransform();
      frameConverter.computePelvisToWorldTransform(markerRigidBody, computedPelvisToWorldTransform);
      RigidBodyTransform actualPelvisToWorldTransform = pelvisFrame.getTransformToWorldFrame();

      assertTrue(computedPelvisToWorldTransform.epsilonEquals(actualPelvisToWorldTransform, 1e-5));
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 100.0)
   @Test(timeout = 10000)
   public void testFrameConversionForRandomPelvisMotion()
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      ReferenceFrame initialPelvisFrame = EuclidFrameRandomTools.nextReferenceFrame("initialPelvisFrame", random, ReferenceFrame.getWorldFrame());
      ReferenceFrame mocapFrame = EuclidFrameRandomTools.nextReferenceFrame("mocapFrame", random, ReferenceFrame.getWorldFrame());
      MocapRigidBody mocapRigidBody = createMocapRigidBody(initialPelvisFrame, mocapFrame);

      MocapToPelvisFrameConverter frameConverter = new MocapToPelvisFrameConverter();
      frameConverter.initialize(initialPelvisFrame, mocapRigidBody);

      for (int i = 0; i < 20; i++)
      {
         ReferenceFrame randomPelvisFrame = EuclidFrameRandomTools.nextReferenceFrame("randomPelvisFrame" + i, random, ReferenceFrame.getWorldFrame());
         MocapRigidBody randomMocapRigidBody = createMocapRigidBody(randomPelvisFrame, mocapFrame);
         RigidBodyTransform computedPelvisToWorldTransform = new RigidBodyTransform();
         frameConverter.computePelvisToWorldTransform(randomMocapRigidBody, computedPelvisToWorldTransform);
         RigidBodyTransform actualPelvisToWorldTransform = randomPelvisFrame.getTransformToWorldFrame();

         assertTrue(actualPelvisToWorldTransform.epsilonEquals(computedPelvisToWorldTransform, 1e-5));
      }
   }

   private MocapRigidBody createMocapRigidBody(ReferenceFrame pelvisFrame, ReferenceFrame mocapFrame)
   {
      RigidBodyTransform pelvisToMarker2Transform = MocapToPelvisFrameConverter.getPelvisToMarker2Transform();
      Vector3D pelvisToMarker2Translation = new Vector3D();
      pelvisToMarker2Transform.getTranslation(pelvisToMarker2Translation);
      pelvisToMarker2Translation.negate();
      FramePoint3D markerPoint2 = new FramePoint3D(pelvisFrame, pelvisToMarker2Translation);
      markerPoint2.changeFrame(mocapFrame);

      RigidBodyTransform pelvisToMocapTransform = new RigidBodyTransform();
      pelvisFrame.getTransformToDesiredFrame(pelvisToMocapTransform, mocapFrame);
      Quaternion pelvisToMocapRotation = new Quaternion();
      pelvisToMocapTransform.getRotation(pelvisToMocapRotation);

      ArrayList<MocapMarker> mocapMarkers = new ArrayList<MocapMarker>();
      mocapMarkers.add(new MocapMarker(1, new Vector3D(), 0.024f));
      mocapMarkers.add(new MocapMarker(2, new Vector3D(), 0.024f));
      mocapMarkers.add(new MocapMarker(3, new Vector3D(), 0.024f));
      mocapMarkers.add(new MocapMarker(4, new Vector3D(), 0.024f));
      return new MocapRigidBody(1, new Vector3D(markerPoint2), pelvisToMocapRotation, mocapMarkers, false);
   }
}
