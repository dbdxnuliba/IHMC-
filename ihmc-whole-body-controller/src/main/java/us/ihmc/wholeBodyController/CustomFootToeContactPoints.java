package us.ihmc.wholeBodyController;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class CustomFootToeContactPoints implements FootContactPoints {

	@Override
	public Map<String, List<Tuple3DBasics>> getSimulationContactPoints(double footLength, double footWidth,
			double toeWidth, DRCRobotJointMap jointMap,
			SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms) {

		HashMap<String, List<Tuple3DBasics>> ret = new HashMap<>();

		for (RobotSide robotSide : RobotSide.values)
		{
			ArrayList<Tuple3DBasics> footContactPoints = new ArrayList<>();
			ArrayList<Tuple3DBasics> toeContactPoints = new ArrayList<>();

			String toeJointName = jointMap.getLegJointName(robotSide, LegJointName.TOE_PITCH);
			String ankleJointName = jointMap.getLegJointName(robotSide, LegJointName.ANKLE_ROLL);

			//SCS Sim contactPoints
			int nContactPointsX = 2;
			int nContactPointsY = 2;

			double dx = 1.01 * footLength / (nContactPointsX - 1.0);
			double xOffset = 1.01 * footLength / 2.0;

			for (int ix = 1; ix <= nContactPointsX; ix++)
			{
				double alpha = (ix - 1.0) / (nContactPointsX - 1.0);
				double footWidthAtCurrentX = (1.0 - alpha) * 1.01 * footWidth + alpha * 1.01 * toeWidth;
				double dy = footWidthAtCurrentX / (nContactPointsY - 1.0);
				double yOffset = footWidthAtCurrentX / 2.0;

				for (int iy = 1; iy <= nContactPointsY; iy++)
				{
					double x = (ix - 1.0) * dx - xOffset;
					double y = (iy - 1.0) * dy - yOffset;
					RigidBodyTransform transformToParentJointFrame = soleToAnkleFrameTransforms.get(robotSide);

					Point3D contactPoint = new Point3D(x, y, 0.0);
					transformToParentJointFrame.transform(contactPoint);

					if(ix == 1)
						footContactPoints.add(contactPoint);
					else
					{
						contactPoint.setZ(-0.032); // adjusting for joint offset
						contactPoint.setX(contactPoint.getX() - 0.01); // tuning
						toeContactPoints.add(contactPoint);
					}
				}
			}

			ret.put(toeJointName, toeContactPoints);
			ret.put(ankleJointName, footContactPoints);
		}

		// adding extra contact points (temporary hack)
		ArrayList<Tuple3DBasics> toeExtraContactPoints = new ArrayList<>();

		// these are the six points around the toe. (already transformed)

		// behind the toe joint
		toeExtraContactPoints.add(new Point3D(-0.04, -0.043, -0.032));
		toeExtraContactPoints.add(new Point3D(-0.04, 0.043, -0.032));       

		// right ahead the toe joint
		toeExtraContactPoints.add(new Point3D(0.04, -0.043, -0.032));
		toeExtraContactPoints.add(new Point3D(0.04, 0.043, -0.032));

		// far forward of the toe.
		toeExtraContactPoints.add(new Point3D( 0.126, -0.043, -0.032 ));
		toeExtraContactPoints.add(new Point3D( 0.126, 0.043, -0.032 ));

		ret.put("l_leg_toe",toeExtraContactPoints);
		ret.put("r_leg_toe",toeExtraContactPoints);

		return ret;

	}

	@Override
	public SideDependentList<List<Tuple2DBasics>> getControllerContactPoints(double footLength, double footWidth, double toeWidth)
	{
		SideDependentList<List<Tuple2DBasics>> ret = new SideDependentList<>();

		for (RobotSide robotSide : RobotSide.values)
		{
			ArrayList<Tuple2DBasics> contactPoints = new ArrayList<>();
			contactPoints.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
			contactPoints.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
			contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
			contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
			ret.put(robotSide, contactPoints);
		}

		return ret;
	}

	@Override
	public SideDependentList<Tuple2DBasics> getToeOffContactPoints(double footLength, double footWidth, double toeWidth)
	{
		SideDependentList<Tuple2DBasics> ret = new SideDependentList<>();

		for (RobotSide robotSide : RobotSide.values)
			ret.put(robotSide, new Point2D(footLength / 2.0, 0.0));

		return ret;
	}

	@Override
	public SideDependentList<LineSegment2D> getToeOffContactLines(double footLength, double footWidth, double toeWidth)
	{
		SideDependentList<LineSegment2D> ret = new SideDependentList<>();

		double footForward = footLength / 2.0;
		double halfToeWidth = toeWidth / 2.0;

		for (RobotSide robotSide : RobotSide.values)
			ret.put(robotSide, new LineSegment2D(new Point2D(footForward, -halfToeWidth), new Point2D(footForward, halfToeWidth)));

		return ret;
	}

	@Override
	public boolean useSoftContactPointParameters()
	{
		return false;
	}

}
