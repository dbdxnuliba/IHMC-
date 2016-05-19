package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygonShrinker;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ICPProportionalController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FrameVector2d tempControl = new FrameVector2d(worldFrame);
   private final YoFrameVector2d icpError = new YoFrameVector2d("icpError", "", worldFrame, registry);
   private final YoFrameVector2d icpErrorIntegrated = new YoFrameVector2d("icpErrorIntegrated", "", worldFrame, registry);

   private final YoFrameVector2d feedbackPart = new YoFrameVector2d("feedbackPart", "", worldFrame, registry);

   private final YoFrameVector2d desiredCMPToICP = new YoFrameVector2d("desiredCMPToICP", "", worldFrame, registry);

   private final YoFrameVector2d rawCMPOutput = new YoFrameVector2d("rawCMPOutput", "", worldFrame, registry);

   private final YoFramePoint icpPosition;
   private final FrameVector2d icpIntegral = new FrameVector2d(worldFrame);

   private final double controlDT;
   private final DoubleYoVariable captureKpParallelToMotion = new DoubleYoVariable("captureKpParallel", registry);
   private final DoubleYoVariable captureKpOrthogonalToMotion = new DoubleYoVariable("captureKpOrthogonal", registry);

   private final DoubleYoVariable captureKi = new DoubleYoVariable("captureKi", registry);
   private final DoubleYoVariable captureKiBleedoff = new DoubleYoVariable("captureKiBleedoff", registry);

   private final Vector2dZUpFrame icpVelocityDirectionFrame;

   private final FrameVector2d tempICPErrorIntegrated = new FrameVector2d(worldFrame);

   private final CMPProjector cmpProjector;

   private final BooleanYoVariable keepCMPInsideSupportPolygon = new BooleanYoVariable("keepCMPInsideSupportPolygon", registry);
   private final DoubleYoVariable maxDistanceCMPSupport = new DoubleYoVariable("maxDistanceCMPSupport", registry);
   private final ConvexPolygonShrinker polygonShrinker = new ConvexPolygonShrinker();
   private final FrameConvexPolygon2d supportPolygonLocal = new FrameConvexPolygon2d();

   public ICPProportionalController(ICPControlGains gains, double controlDT, CMPProjector cmpProjector,
         double maxAllowedDistanceCMPSupport, YoVariableRegistry parentRegistry)
   {
      this.cmpProjector = cmpProjector;

      this.controlDT = controlDT;

      icpVelocityDirectionFrame = new Vector2dZUpFrame("icpVelocityDirectionFrame", worldFrame);

      icpPosition = new YoFramePoint("icpPosition", ReferenceFrame.getWorldFrame(), registry);
      parentRegistry.addChild(registry);

      captureKpParallelToMotion.set(gains.getKpParallelToMotion());
      captureKpOrthogonalToMotion.set(gains.getKpOrthogonalToMotion());
      captureKi.set(gains.getKi());
      captureKiBleedoff.set(gains.getKiBleedOff());

      keepCMPInsideSupportPolygon.set(false);
      maxDistanceCMPSupport.set(maxAllowedDistanceCMPSupport);
   }

   public void reset()
   {
      icpErrorIntegrated.set(0.0, 0.0);
   }

   private final FramePoint2d desiredCMP = new FramePoint2d();

   public FramePoint2d doProportionalControl(FramePoint2d capturePoint, FramePoint2d desiredCapturePoint, FramePoint2d finalDesiredCapturePoint,
         FrameVector2d desiredCapturePointVelocity, double omega0, FrameConvexPolygon2d supportPolygon)
   {
      capturePoint.changeFrame(worldFrame);
      desiredCapturePoint.changeFrame(worldFrame);
      finalDesiredCapturePoint.changeFrame(worldFrame);
      desiredCapturePointVelocity.changeFrame(worldFrame);

      desiredCMP.setIncludingFrame(capturePoint);

      icpPosition.set(capturePoint.getX(), capturePoint.getY(), 0.0);
      // feed forward part
      tempControl.setIncludingFrame(desiredCapturePointVelocity);
      tempControl.scale(1.0 / omega0);

      desiredCMP.sub(tempControl);

      // feedback part
      icpError.set(capturePoint);
      icpError.sub(desiredCapturePoint);

      icpError.getFrameTuple2d(tempControl);
      double epsilonZeroICPVelocity = 1e-5;
      if (desiredCapturePointVelocity.lengthSquared() > MathTools.square(epsilonZeroICPVelocity))
      {
         icpVelocityDirectionFrame.setXAxis(desiredCapturePointVelocity);
         tempControl.changeFrame(icpVelocityDirectionFrame);
         tempControl.setX(tempControl.getX() * captureKpParallelToMotion.getDoubleValue());
         tempControl.setY(tempControl.getY() * captureKpOrthogonalToMotion.getDoubleValue());
         tempControl.changeFrame(desiredCMP.getReferenceFrame());
      }
      else
      {
         tempControl.scale(captureKpOrthogonalToMotion.getDoubleValue());
      }

      icpError.getFrameTuple2d(tempICPErrorIntegrated);
      tempICPErrorIntegrated.scale(controlDT);
      tempICPErrorIntegrated.scale(captureKi.getDoubleValue());

      icpErrorIntegrated.scale(captureKiBleedoff.getDoubleValue());
      icpErrorIntegrated.add(tempICPErrorIntegrated);

      double length = icpErrorIntegrated.length();
      double maxLength = 0.02;
      if (length > maxLength)
      {
         icpErrorIntegrated.scale(maxLength / length);
      }

      if (Math.abs(captureKi.getDoubleValue()) < 1e-10)
      {
         icpErrorIntegrated.set(0.0, 0.0);
      }

      icpErrorIntegrated.getFrameTuple2d(icpIntegral);
      tempControl.add(icpIntegral);

      feedbackPart.set(tempControl);
      desiredCMP.add(tempControl);

      desiredCMPToICP.sub(capturePoint, desiredCMP);

      supportPolygonLocal.setIncludingFrame(supportPolygon);
      if (!keepCMPInsideSupportPolygon.getBooleanValue())
      {
         polygonShrinker.shrinkConstantDistanceInto(supportPolygon, -maxDistanceCMPSupport.getDoubleValue(), supportPolygonLocal);
      }

      cmpProjector.projectCMPIntoSupportPolygonIfOutside(capturePoint, supportPolygonLocal, finalDesiredCapturePoint, desiredCMP);
      capturePoint.changeFrame(worldFrame);
      desiredCMP.changeFrame(worldFrame);

      if (desiredCMP.containsNaN())
      {
         desiredCMP.set(capturePoint);
         System.err.println("ICPProportionalController: desiredCMP contained NaN. Set it to capturePoint...");
      }
      if (cmpProjector.getWasCMPProjected())
      {
         icpErrorIntegrated.scale(0.9); //Bleed off quickly when projecting. 0.9 is a pretty arbitrary magic number.
      }

      desiredCMP.changeFrame(rawCMPOutput.getReferenceFrame());
      rawCMPOutput.set(desiredCMP);

      return desiredCMP;
   }

   private class Vector2dZUpFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -1810366869361449743L;
      private final FrameVector2d xAxis;
      private final Vector3d x = new Vector3d();
      private final Vector3d y = new Vector3d();
      private final Vector3d z = new Vector3d();
      private final Matrix3d rotation = new Matrix3d();

      public Vector2dZUpFrame(String string, ReferenceFrame parentFrame)
      {
         super(string, parentFrame);
         xAxis = new FrameVector2d(parentFrame);
      }

      public void setXAxis(FrameVector2d xAxis)
      {
         this.xAxis.setIncludingFrame(xAxis);
         this.xAxis.changeFrame(parentFrame);
         this.xAxis.normalize();
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         x.set(xAxis.getX(), xAxis.getY(), 0.0);
         z.set(0.0, 0.0, 1.0);
         y.cross(z, x);

         rotation.setColumn(0, x);
         rotation.setColumn(1, y);
         rotation.setColumn(2, z);

         transformToParent.setRotationAndZeroTranslation(rotation);
      }
   }

   public void setKeepCMPInsideSupportPolygon(boolean keepCMPInsideSupportPolygon)
   {
      this.keepCMPInsideSupportPolygon.set(keepCMPInsideSupportPolygon);
   }
}
