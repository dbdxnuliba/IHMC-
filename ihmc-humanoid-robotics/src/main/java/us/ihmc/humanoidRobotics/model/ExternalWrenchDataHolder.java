package us.ihmc.humanoidRobotics.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;

public class ExternalWrenchDataHolder
{
   private final Map<String, RigidBody> nameToRigidBodyMap = new LinkedHashMap<>();
   private final Map<RigidBody, Wrench> externalWrenches = new LinkedHashMap<>();
   private final List<Wrench> allWrenches = new ArrayList<>();

   public ExternalWrenchDataHolder(Collection<RigidBody> rigidBodies)
   {
      for (RigidBody rigidBody : rigidBodies)
      {
         nameToRigidBodyMap.put(rigidBody.getName(), rigidBody);
         Wrench externalWrench = new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
         externalWrenches.put(rigidBody, externalWrench);
         allWrenches.add(externalWrench);
      }
   }

   public void setToZero()
   {
      for (int i = 0; i < allWrenches.size(); i++)
      {
         allWrenches.get(i).setToZero();
      }
   }

   public void setExternalWrench(RigidBody rigidBody, Wrench externalWrench)
   {
      Wrench localWrench = externalWrenches.get(rigidBody);
      if (localWrench == null)
      {
         nameToRigidBodyMap.put(rigidBody.getName(), rigidBody);
         localWrench = new Wrench(rigidBody.getBodyFixedFrame(), rigidBody.getBodyFixedFrame());
         externalWrenches.put(rigidBody, localWrench);
         allWrenches.add(localWrench);
      }
      localWrench.checkAndSet(externalWrench);
   }

   public void getExternalWrench(RigidBody rigidBody, Wrench externalWrenchToPack)
   {
      externalWrenchToPack.set(externalWrenches.get(rigidBody));
   }
}
