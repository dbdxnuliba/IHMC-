package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.scene.Group;
import javafx.scene.paint.Color;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.OrientationGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.PoseEditable;

public class PatrolWaypointGraphic extends Group implements PoseEditable
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final FramePose3D pose = new FramePose3D();

   private final PositionGraphic snappedPositionGraphic;
   private final OrientationGraphic orientationGraphic;
   private LabelGraphic labelGraphic;

   public PatrolWaypointGraphic(int index)
   {
      snappedPositionGraphic = new PositionGraphic(Color.YELLOW, 0.05);
      orientationGraphic = new OrientationGraphic();
      labelGraphic = new LabelGraphic(String.valueOf(index));

      getChildren().add(snappedPositionGraphic.getNode());
      getChildren().add(orientationGraphic.getNode());
      getChildren().add(labelGraphic.getNode());
   }

   @Override
   public void setPosition(Point3DReadOnly position)
   {
      pose.setPosition(position);
      updateGraphics();
   }

   @Override
   public void setOrientation(Orientation3DReadOnly orientationPoint)
   {
      pose.setOrientation(orientationPoint);
      updateGraphics();
   }

   public PositionGraphic getSnappedPositionGraphic()
   {
      return snappedPositionGraphic;
   }

   public OrientationGraphic getOrientationGraphic()
   {
      return orientationGraphic;
   }

   public void redrawIndex(int index)
   {
      getChildren().remove(labelGraphic.getNode());
      labelGraphic = new LabelGraphic(String.valueOf(index));
      getChildren().add(labelGraphic.getNode());
      updateGraphics();
   }

   @Override
   public Point3DBasics getPosition()
   {
      return pose.getPosition();
   }

   public Orientation3DReadOnly getOrientation()
   {
      return pose.getOrientation();
   }

   private void updateGraphics()
   {
      snappedPositionGraphic.getPose().set(pose);
      snappedPositionGraphic.update();

      orientationGraphic.getPose().set(pose);
      orientationGraphic.update();

      labelGraphic.getPose().set(pose);
      labelGraphic.update();
   }
}
