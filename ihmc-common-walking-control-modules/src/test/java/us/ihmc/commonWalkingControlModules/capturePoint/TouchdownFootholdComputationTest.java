package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

public class TouchdownFootholdComputationTest
{
   private final Random random = new Random(4280839L);

   @Test
   public void testFootholdPlaneProjection()
   {
      for (int i = 0; i < 1000; i++)
      {
         FramePose3D solePose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame());
         FramePose3D footholdPose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame());
         Vector2D soleToHeel = EuclidCoreRandomTools.nextVector2D(random);

         FramePose3D adjustedSole = new FramePose3D(solePose);
         computeAdjustedSole(adjustedSole, footholdPose, soleToHeel);

         PoseReferenceFrame footholdFrame = new PoseReferenceFrame("FootholdFrame", footholdPose);
         PoseReferenceFrame soleFrame = new PoseReferenceFrame("SoleFrame", solePose);
         PoseReferenceFrame adjustedSoleFrame = new PoseReferenceFrame("AdjustedSoleFrame", adjustedSole);

         FramePoint3D heelExpected = new FramePoint3D(soleFrame, soleToHeel);
         FramePoint3D heelActual = new FramePoint3D(adjustedSoleFrame, soleToHeel);
         FrameVector3D zExpected = new FrameVector3D(footholdFrame, 0.0, 0.0, 1.0);
         FrameVector3D zActual = new FrameVector3D(adjustedSoleFrame, 0.0, 0.0, 1.0);

         heelExpected.changeFrame(ReferenceFrame.getWorldFrame());
         heelActual.changeFrame(ReferenceFrame.getWorldFrame());
         EuclidCoreTestTools.assertTuple3DEquals(heelExpected, heelActual, 1.0e-10);

         zExpected.changeFrame(ReferenceFrame.getWorldFrame());
         zActual.changeFrame(ReferenceFrame.getWorldFrame());
         EuclidCoreTestTools.assertTuple3DEquals(zExpected, zActual, 1.0e-10);

         // TODO: Is there a good check on the yaw? Actually checking the yaw does not work.
      }
   }

   private final Quaternion orientationError = new Quaternion();
   private final Quaternion rotation = new Quaternion();
   private final Vector3DReadOnly zAxis = new Vector3D(0.0, 0.0, 1.0);

   /**
    * This method projects the sole frame onto the x-y plane of the foothold pose. It preserves the heel location of the
    * sole and attempts to avoid modifying the yaw of the sole in world.
    *
    * @param solePose to be projected (modified)
    * @param footholdPose defines the x-y plane
    * @param soleToHeel heel location in sole frame
    */
   private void computeAdjustedSole(FixedFramePose3DBasics solePose, FramePose3DReadOnly footholdPose, Vector2D soleToHeel)
   {
      solePose.checkReferenceFrameMatch(footholdPose);

      orientationError.set(solePose.getOrientation());
      orientationError.multiplyConjugateThis(footholdPose.getOrientation());
      EuclidCoreMissingTools.projectRotationOnAxis(orientationError, zAxis, rotation);
      rotation.conjugate();

      solePose.appendTranslation(soleToHeel.getX(), soleToHeel.getY(), 0.0);
      solePose.appendRotation(orientationError);
      solePose.appendRotation(rotation);
      solePose.appendTranslation(-soleToHeel.getX(), -soleToHeel.getY(), 0.0);
   }
}
