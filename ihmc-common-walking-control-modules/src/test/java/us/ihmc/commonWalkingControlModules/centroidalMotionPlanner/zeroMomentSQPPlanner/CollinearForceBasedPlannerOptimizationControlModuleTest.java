package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.zeroMomentSQPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import com.sun.media.jfxmedia.events.NewFrameEvent;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class CollinearForceBasedPlannerOptimizationControlModuleTest
{
   @Test
   public void testSmoothnessConstraints()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName() + "Registry");
      YoInteger numberOfPlanningSegments = new YoInteger("NumberOfPlanningSegments", registry);
      FrameVector3D gravity = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -9.81);
      CollinearForceBasedPlannerIterationResult sqpSolution = new CollinearForceBasedPlannerIterationResult(gravity);
      CollinearForceBasedPlannerOptimizationControlModule optimizationControlModule = new CollinearForceBasedPlannerOptimizationControlModule(sqpSolution,
                                                                                                                                              numberOfPlanningSegments,
                                                                                                                                              gravity,
                                                                                                                                              registry);
      optimizationControlModule.reset();
      List<CollinearForceMotionPlannerSegment> segmentList = new ArrayList<>();
      for (int i = 0; i < 4; i++)
      {
         CollinearForceMotionPlannerSegment segment1 = new CollinearForceMotionPlannerSegment();
         segment1.setSegmentDuration(0.15);
         segmentList.add(segment1);
         sqpSolution.comTrajectories.add().setSeptic(0.0, 0.05, 0.1, 0.15, new FramePoint3D(), new FrameVector3D(), new FramePoint3D(), new FrameVector3D(),
                                                     new FramePoint3D(), new FrameVector3D(), new FramePoint3D(), new FrameVector3D());
         sqpSolution.copTrajectories.add().setSeptic(0.0, 0.05, 0.1, 0.15, new FramePoint3D(), new FrameVector3D(), new FramePoint3D(), new FrameVector3D(),
                                                     new FramePoint3D(), new FrameVector3D(), new FramePoint3D(), new FrameVector3D());
         sqpSolution.scalarProfile.add().setCubic(0.0, 0.15, 28.0f, 28.0f);
      }
      optimizationControlModule.submitSegmentList(segmentList);
      optimizationControlModule.compute();
      PrintTools.debug(optimizationControlModule.toString());
   }

}
