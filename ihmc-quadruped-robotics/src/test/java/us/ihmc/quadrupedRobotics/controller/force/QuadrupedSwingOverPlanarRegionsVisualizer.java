package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.Tag;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander.SwingOverPlanarRegionsTrajectoryCollisionType;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.graphics.Graphics3DObjectTools;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

import java.util.HashMap;
import java.util.Map;

@Tag("humanoid-rough-terrain")
public class QuadrupedSwingOverPlanarRegionsVisualizer
{
   private static final double swingHeight = 0.08;

   private static final ReferenceFrame WORLD = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.Gray(), YoAppearance.Gray()};

   private final SimulationConstructionSet scs;
   private final YoVariableRegistry registry;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final YoFramePoint3D startPosition;
   private final YoFramePoint3D endPosition;
   private final YoFramePoint3D solePosition;
   private final YoGraphicPosition collisionSphere;
   private final YoGraphicPosition swingStartGraphic;
   private final YoGraphicPosition swingEndGraphic;
   private final Map<SwingOverPlanarRegionsTrajectoryCollisionType, YoGraphicPosition> intersectionMap;

   private final QuadrupedPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;

   public QuadrupedSwingOverPlanarRegionsVisualizer(SimulationConstructionSet scs, YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.scs = scs;
      this.registry = registry;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;

      swingOverPlanarRegionsTrajectoryExpander = new QuadrupedPlanarRegionsTrajectoryExpander(registry, yoGraphicsListRegistry);
      swingOverPlanarRegionsTrajectoryExpander.attachVisualizer(this::update);

      startPosition = new YoFramePoint3D("startPosition", WORLD, registry);
      endPosition= new YoFramePoint3D("endPosition", WORLD, registry);
      solePosition = new YoFramePoint3D("solePoint", WORLD, registry);
      AppearanceDefinition bubble = YoAppearance.LightBlue();
      bubble.setTransparency(0.5);
      collisionSphere = new YoGraphicPosition("CollisionSphere", solePosition, swingOverPlanarRegionsTrajectoryExpander.getSphereRadius(), bubble);
      swingStartGraphic = new YoGraphicPosition("SwingStartGraphic", startPosition, 0.03, YoAppearance.Green());
      swingEndGraphic = new YoGraphicPosition("SwingEndGraphic", endPosition, 0.03, YoAppearance.Yellow());
      intersectionMap = new HashMap<>();
      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         AppearanceDefinition appearance;
         double size;
         switch (swingOverPlanarRegionsTrajectoryCollisionType)
         {
         case CRITICAL_INTERSECTION:
            appearance = YoAppearance.Red();
            size = 0.014;
            break;
         case INTERSECTION_BUT_OUTSIDE_TRAJECTORY:
            appearance = YoAppearance.Orange();
            size = 0.013;
            break;
         case INTERSECTION_BUT_BELOW_IGNORE_PLANE:
            appearance = YoAppearance.Yellow();
            size = 0.012;
            break;
         case NO_INTERSECTION:
            appearance = YoAppearance.Blue();
            size = 0.011;
            break;
         default:
            appearance = YoAppearance.Black();
            size = 0.01;
            break;
         }
         intersectionMap.put(swingOverPlanarRegionsTrajectoryCollisionType,
                             new YoGraphicPosition("IntersectionGraphic" + swingOverPlanarRegionsTrajectoryCollisionType.name(),
                                                   new YoFramePoint3D("IntersectionPoint" + swingOverPlanarRegionsTrajectoryCollisionType.name(), WORLD,
                                                                    registry),
                                                   size, appearance));

         yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", collisionSphere);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingStartGraphic);
      yoGraphicsListRegistry.registerYoGraphic("SwingOverPlanarRegions", swingEndGraphic);

   }

   public QuadrupedSwingOverPlanarRegionsVisualizer()
   {
      this(new SimulationConstructionSet(new Robot("Robot")), new YoVariableRegistry(QuadrupedSwingOverPlanarRegionsVisualizer.class.getSimpleName()),
           new YoGraphicsListRegistry());

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(1.0, 1);
      scs.setCameraFix(0.4, 0.0, 0.0);

      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.4, 0.0, 0.0001);
      generator.addCubeReferencedAtBottomMiddle(1.0, 1.0, 0.001);
      generator.translate(-0.15, 0.0, 0.0001);
      generator.translate(0.0, 0.0, 0.25);
      generator.addRectangle(0.1, 0.1);
//      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);
      generator.translate(0.52, -0.12, 0.1);
      generator.addCubeReferencedAtBottomMiddle(0.1, 0.1, 0.1);

      PlanarRegionsList terrain = generator.getPlanarRegionsList();
      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      Graphics3DObjectTools.addPlanarRegionsList(graphics3DObject, terrain, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      startPosition.set(0.0, 0.0, 0.0);
      endPosition.set(0.8, 0.0, 0.0);
      swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(startPosition, endPosition, swingHeight, terrain, null);

      scs.startOnAThread();
      scs.cropBuffer();
      ThreadTools.sleepForever();
   }

   public double expandTrajectoryOverPlanarRegions(FramePoint3DReadOnly swingStartPose, FramePoint3DReadOnly swingEndPose,
                                                   PlanarRegionsList planarRegionsList)
   {
      this.startPosition.set(swingStartPose);
      this.endPosition.set(swingEndPose);
      return swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(swingStartPose, swingEndPose, swingHeight, planarRegionsList, null);
   }

   public QuadrupedPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return swingOverPlanarRegionsTrajectoryExpander;
   }

   private void update(double dt)
   {
      solePosition.set(swingOverPlanarRegionsTrajectoryExpander.getTrajectoryPosition());

      for (SwingOverPlanarRegionsTrajectoryCollisionType swingOverPlanarRegionsTrajectoryCollisionType : SwingOverPlanarRegionsTrajectoryCollisionType.values())
      {
         intersectionMap.get(swingOverPlanarRegionsTrajectoryCollisionType)
                        .setPosition(swingOverPlanarRegionsTrajectoryExpander.getClosestPolygonPoint(swingOverPlanarRegionsTrajectoryCollisionType));
      }

      collisionSphere.update();

      scs.tickAndUpdate(scs.getTime() + dt);
   }
}
