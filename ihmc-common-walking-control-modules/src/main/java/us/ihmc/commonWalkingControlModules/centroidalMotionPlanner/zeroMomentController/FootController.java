package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FootController
{
   public enum FootControlMode
   {
      FREE_MOTION, CONTACT
   }

   private final YoVariableRegistry registry;
   private final YoEnum<FootControlMode> currentState;
   private final YoEnum<FootControlMode> requestedState;
   private final FootSupportPolygon supportPolygon;
   private final ReferenceFrame soleFrame;
   private final YoDouble maxNormalForce;
   private final List<Point2DReadOnly> currentContactPoints = new ArrayList<>();
   private final List<Point2DReadOnly> desiredContactPoints = new ArrayList<>();

   public FootController(ReferenceFrame soleFrame, FootSupportPolygon supportPolygon, RobotSide side, YoVariableRegistry parentRegistry)
   {
      String footName = side.getCamelCaseNameForStartOfExpression() + "Foot";
      registry = new YoVariableRegistry(side.getCamelCaseNameForMiddleOfExpression() + "FootController");
      this.supportPolygon = supportPolygon;
      this.soleFrame = soleFrame;
      currentState = new YoEnum<>(footName + "ControlState", registry, FootControlMode.class);
      requestedState = new YoEnum<>(footName + "RequestedControlState", registry, FootControlMode.class);
      maxNormalForce = new YoDouble(footName + "MaxNormalForce", registry);
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      currentState.set(FootControlMode.CONTACT);
   }

   public void requestFreeMotion()
   {
      requestedState.set(FootControlMode.FREE_MOTION);
      computeRhoRampingProfileForLiftOff();
   }

   public void requestContact(boolean touchdownAtToes, boolean touchdownAtHeel)
   {
      requestedState.set(FootControlMode.CONTACT);
      if(touchdownAtHeel)
         supportPolygon.getHeelVertices(desiredContactPoints);
      else if(touchdownAtToes)
         supportPolygon.getToeVertices(desiredContactPoints);
      else
         supportPolygon.getVertices(desiredContactPoints);
      computeRhoRampingProfileForTouchdown();
   }

   public void doControl()
   {
      if (currentState.getEnumValue() != requestedState.getEnumValue())
      {
         // handle transitions 
      }
   }

   private void computeRhoRampingProfileForLiftOff()
   {

   }

   private void computeRhoRampingProfileForTouchdown()
   {

   }
}
