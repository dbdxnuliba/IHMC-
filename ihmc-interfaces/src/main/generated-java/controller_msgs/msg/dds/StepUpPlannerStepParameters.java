package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class StepUpPlannerStepParameters extends Packet<StepUpPlannerStepParameters> implements Settable<StepUpPlannerStepParameters>, EpsilonComparable<StepUpPlannerStepParameters>
{
   /**
          * Definition of states
          */
   public static final byte STAND = (byte) 0;
   public static final byte SWING = (byte) 1;
   /**
            * Type of step
            */
   public byte state_;
   /**
            * Vertices of the left foot for every phase
            * (in case of SWING state this is ignored)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerVector2>  foot_vertices_;

   public StepUpPlannerStepParameters()
   {
      foot_vertices_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerVector2> (10, new controller_msgs.msg.dds.StepUpPlannerVector2PubSubType());

   }

   public StepUpPlannerStepParameters(StepUpPlannerStepParameters other)
   {
      this();
      set(other);
   }

   public void set(StepUpPlannerStepParameters other)
   {
      state_ = other.state_;

      foot_vertices_.set(other.foot_vertices_);
   }

   /**
            * Type of step
            */
   public void setState(byte state)
   {
      state_ = state;
   }
   /**
            * Type of step
            */
   public byte getState()
   {
      return state_;
   }


   /**
            * Vertices of the left foot for every phase
            * (in case of SWING state this is ignored)
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.StepUpPlannerVector2>  getFootVertices()
   {
      return foot_vertices_;
   }


   public static Supplier<StepUpPlannerStepParametersPubSubType> getPubSubType()
   {
      return StepUpPlannerStepParametersPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return StepUpPlannerStepParametersPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(StepUpPlannerStepParameters other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.state_, other.state_, epsilon)) return false;

      if (this.foot_vertices_.size() != other.foot_vertices_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.foot_vertices_.size(); i++)
         {  if (!this.foot_vertices_.get(i).epsilonEquals(other.foot_vertices_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof StepUpPlannerStepParameters)) return false;

      StepUpPlannerStepParameters otherMyClass = (StepUpPlannerStepParameters) other;

      if(this.state_ != otherMyClass.state_) return false;

      if (!this.foot_vertices_.equals(otherMyClass.foot_vertices_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("StepUpPlannerStepParameters {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("foot_vertices=");
      builder.append(this.foot_vertices_);
      builder.append("}");
      return builder.toString();
   }
}
