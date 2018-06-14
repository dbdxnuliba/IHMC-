package us.ihmc.avatar.collinearVisualizer;

import java.awt.Color;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentController.CentroidalState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class CentroidalRobotControllerOutputWriter implements RobotController
{
   protected final YoVariableRegistry registry = new YoVariableRegistry("ControllerRegistry");
   private final CentroidalState state;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ExternalForcePoint forcePoint;
   private final YoFramePoint3D cop;
   private final YoFrameVector3D force;
   private final CentroidalRobotController controller;

   public CentroidalRobotControllerOutputWriter(String robotName, CentroidalRobotController controller, FloatingJoint floatingJoint, YoGraphicsListRegistry graphicsListRegistry)
   {
      String namePrefix = robotName + "OutputWriter";
      state = new CentroidalState(namePrefix + "State", registry);
      
      forcePoint = new ExternalForcePoint("ForcePoint", registry);
      force = new YoFrameVector3D(namePrefix + "Force", worldFrame, registry);
      cop = new YoFramePoint3D(namePrefix + "CoP", worldFrame, registry);
      floatingJoint.addExternalForcePoint(forcePoint);

      this.controller = controller;
      registry.addChild(controller.getRegistry());
      controller.setOutputVariables(cop, force);
      controller.setCurrentState(state);
      
      if (graphicsListRegistry != null)
      {
         YoGraphicVector forceGraphic = new YoGraphicVector(namePrefix + "ForceGraphic", cop.getYoX(), cop.getYoY(), cop.getYoZ(), force.getYoX(),
                                                            force.getYoY(), force.getYoZ(), 0.002, new YoAppearanceRGBColor(Color.RED, 0.0), true);
         YoGraphicPosition copGraphic = new YoGraphicPosition(namePrefix + "CoPGraphic", cop, 0.002, new YoAppearanceRGBColor(Color.ORANGE, 0.0));
         graphicsListRegistry.registerArtifact("OutputWriterArtifactsList", copGraphic.createArtifact());
         graphicsListRegistry.registerYoGraphic("OutputWriterGraphicsList", forceGraphic);
         graphicsListRegistry.registerYoGraphic("OutputWriterGraphicsList", copGraphic);
      }
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return "CentroidalRobotController";
   }

   @Override
   public String getDescription()
   {
      return "Exerts a force from an external point on the root joint of robot";
   }

   public void finalizeControlInputs()
   {
      forcePoint.setOffsetWorld(cop.getX(), cop.getY(), cop.getZ());
      forcePoint.setForce(force.getX(), force.getY(), force.getZ());
      forcePoint.active = true;
   }

   public CentroidalState getState()
   {
      return state;
   }

   @Override
   public void initialize()
   {
      controller.initialize();
   }

   @Override
   public void doControl()
   {
      controller.doControl();
      finalizeControlInputs();
   }
}