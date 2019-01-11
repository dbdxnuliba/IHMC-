package us.ihmc.manipulation.planning.planarRobot;

import gnu.trove.list.array.TDoubleArrayList;

public interface InverseKinematicsInterface
{
   abstract void setInitialConfiguration(TDoubleArrayList jointConfiguration);

   abstract void setDesiredTaskConfiguration(TDoubleArrayList taskConfiguration);

   abstract boolean solve();

   abstract TDoubleArrayList getSolution();
}
