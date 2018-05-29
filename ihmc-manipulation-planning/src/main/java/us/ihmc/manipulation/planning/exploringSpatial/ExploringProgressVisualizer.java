package us.ihmc.manipulation.planning.exploringSpatial;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class ExploringProgressVisualizer
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private double recentProgress;
   private double advancedProgress = 0.0;

   private boolean desiredNodeValidity;

   private static Point3D pointRecentProgressLineOrigin = new Point3D(0.0, 1.0, 1.0);
   private Point3D pointRecentProgress = new Point3D(0.0, 0.0, 1.0);
   private Point3D pointAdvancedProgress = new Point3D(0.0, 0.0, 1.0);

   private final YoFramePoint3D yoFramePointRecentProgressLineOrigin;
   private final YoFramePoint3D yoFramePointRecentProgress;
   private final YoFramePoint3D yoFramePointRecentProgressInvalid;
   private final YoFramePoint3D yoFramePointAdvancedProgress;

   /*
    * find sphere thing
    */
   private final YoGraphicCylinder recentProgressLineViz;
   private final YoGraphicPosition recentProgressViz;
   private final YoGraphicPosition recentProgressInvalidViz;
   private final YoGraphicPosition advancedProgressViz;

   public ExploringProgressVisualizer(String name, String graphicsListName, YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList("TreeStateVisualizerGraphicsList");

      /*
       * set recentProgressLine.
       */
      yoFramePointRecentProgressLineOrigin = new YoFramePoint3D(name + "recentProgressOrigin", worldFrame, registry);
      yoFramePointRecentProgressLineOrigin.set(pointRecentProgressLineOrigin);

      YoFrameVector3D recentProgressLineVector = new YoFrameVector3D("recentProgressLineVector", worldFrame, registry);
      recentProgressLineVector.set(new Vector3D(0.0, 1.0, 0.0));

      recentProgressLineViz = new YoGraphicCylinder("recentProgressLine", yoFramePointRecentProgressLineOrigin, recentProgressLineVector,
                                                    YoAppearance.LightBlue(), 0.03);

      yoGraphicsList.add(recentProgressLineViz);

      /*
       * set recentProgress.
       */
      yoFramePointRecentProgress = new YoFramePoint3D(name + "recentProgress", worldFrame, registry);
      pointRecentProgress.set(pointRecentProgressLineOrigin);
      yoFramePointRecentProgress.set(pointRecentProgress);

      yoFramePointRecentProgressInvalid = new YoFramePoint3D(name + "recentProgressInvalid", worldFrame, registry);
      pointRecentProgress.set(pointRecentProgressLineOrigin);
      yoFramePointRecentProgressInvalid.set(pointRecentProgress);

      recentProgressViz = new YoGraphicPosition("recentProgress", yoFramePointRecentProgress, 0.06, YoAppearance.Blue(), GraphicType.BALL);
      yoGraphicsList.add(recentProgressViz);

      recentProgressInvalidViz = new YoGraphicPosition("recentProgressInvalid", yoFramePointRecentProgressInvalid, 0.06, YoAppearance.Red(), GraphicType.BALL);
      yoGraphicsList.add(recentProgressInvalidViz);

      /*
       * set advancedProgress.
       */
      yoFramePointAdvancedProgress = new YoFramePoint3D(name + "advancedProgress", worldFrame, registry);
      pointRecentProgress.set(pointRecentProgressLineOrigin);
      yoFramePointAdvancedProgress.set(pointRecentProgress);

      advancedProgressViz = new YoGraphicPosition("advancedProgress", yoFramePointAdvancedProgress, 0.05, YoAppearance.Black(), GraphicType.BALL);
      yoGraphicsList.add(advancedProgressViz);

      /*
       * register YoGraphicsList
       */
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }

   public void setRecentProgress(double value)
   {
      recentProgress = value;
      if (recentProgress > advancedProgress)
         advancedProgress = value;
   }

   public void setDesiredNodeValidity(boolean value)
   {
      desiredNodeValidity = value;
   }

   public void updateVisualizer()
   {
      if (desiredNodeValidity)
      {
         pointRecentProgress.set(pointRecentProgressLineOrigin);
         pointRecentProgress.add(new Vector3D(0.0, recentProgress, 0.0));
         yoFramePointRecentProgress.set(pointRecentProgress);

         pointRecentProgress.setToNaN();
         yoFramePointRecentProgressInvalid.set(pointRecentProgress);
      }
      else
      {
         pointRecentProgress.set(pointRecentProgressLineOrigin);
         pointRecentProgress.add(new Vector3D(0.0, recentProgress, 0.0));
         yoFramePointRecentProgressInvalid.set(pointRecentProgress);

         pointRecentProgress.setToNaN();
         yoFramePointRecentProgress.set(pointRecentProgress);
      }

      pointAdvancedProgress.set(pointRecentProgressLineOrigin);
      pointAdvancedProgress.add(new Vector3D(0.0, advancedProgress, 0.0));
      yoFramePointAdvancedProgress.set(pointAdvancedProgress);

      recentProgressInvalidViz.update();
      advancedProgressViz.update();
      recentProgressLineViz.update();
      recentProgressViz.update();
   }
}