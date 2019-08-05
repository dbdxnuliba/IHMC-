package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import controller_msgs.msg.dds.CollisionAvoidanceManagerMessage;
import controller_msgs.msg.dds.PlanarRegionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.collisionAvoidance.CollisionAvoidanceMessageMode;

public class CollisionAvoidanceManagerCommand implements Command<CollisionAvoidanceManagerCommand, CollisionAvoidanceManagerMessage>
{

   private final PlanarRegionCommand planarRegionCommand = new PlanarRegionCommand();
   private final RecyclingArrayList<PlanarRegionCommand> planarRegions = new RecyclingArrayList<>(100, PlanarRegionCommand.class);
   private CollisionAvoidanceMessageMode mode = CollisionAvoidanceMessageMode.OVERRIDE;
   private long sequenceId;
   private final AtomicBoolean considerOnlyEdges = new AtomicBoolean(false);


   @Override
   public void set(CollisionAvoidanceManagerCommand other)
   {
      clear();

      mode = other.mode;
      sequenceId = other.sequenceId;
      considerOnlyEdges.set(other.considerOnlyEdges.get());

      RecyclingArrayList<PlanarRegionCommand> dataList = other.getPlanarRegions();
      if (dataList != null)
      {
         for (int i = 0; i < dataList.size(); i++)
            planarRegions.add().set(dataList.get(i));
      }
   }

   public int getNumberOfPlanarRegions()
   {
      return planarRegions.size();
   }

   public RecyclingArrayList<PlanarRegionCommand> getPlanarRegions()
   {
      return planarRegions;
   }

   public PlanarRegionCommand getPlanarRegionCommand(int i)
   {
      return planarRegions.get(i);
   }

   public CollisionAvoidanceMessageMode getMode()
   {
      return mode;
   }

   public boolean considerOnlyEdges()
   {
      return considerOnlyEdges.get();
   }

   @Override
   public void clear()
   {
      mode = CollisionAvoidanceMessageMode.OVERRIDE;
      planarRegions.clear();
      sequenceId = 0;
      considerOnlyEdges.set(false);
   }

   @Override
   public void setFromMessage(CollisionAvoidanceManagerMessage message)
   {
      clear();

      sequenceId = message.getSequenceId();
      mode = CollisionAvoidanceMessageMode.fromByte(message.getMode());
      considerOnlyEdges.set(message.getConsiderOnlyEdges());

      List<PlanarRegionMessage> planarRegionsList = message.getPlanarRegionsList();
      
      for (int i = 0; i < planarRegionsList.size(); ++i) 
      {
         planarRegionCommand.setFromMessage(planarRegionsList.get(i));
         planarRegions.add().set(planarRegionCommand);
      }

   }

   @Override
   public Class<CollisionAvoidanceManagerMessage> getMessageClass()
   {
      return CollisionAvoidanceManagerMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

}
