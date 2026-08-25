#include "perception_layer.h"

namespace perception
{
  TfAccessor::TfAccessor(ros::NodeHandle &bt_nh , Subscriber &subscriber) : tf_listener_(tf_buffer_) , bt_nh_(bt_nh) , subscriber_(subscriber)
  {

  }

  geometry_msgs::TransformStamped TfAccessor::getTfTransform(const FrameId &target_frame,const FrameId &source_frame) const
  {
    ROS_ASSERT(target_frame != FrameId::TRACK);
    if (source_frame != FrameId::TRACK){
      return tf_buffer_.lookupTransform(frame_map.at(target_frame),
                                        frame_map.at(source_frame), ros::Time(0), ros::Duration(0.05));
    }else
    {
      return tf_buffer_.lookupTransform(frame_map.at(target_frame),subscriber_.msgGetter<rm_msgs::TrackData>(perception::Subscriber::TopicId::TRACK_DATA).message.header.frame_id,ros::Time(0),ros::Duration(0.05));
    }
  }

  Publisher::Publisher(ros::NodeHandle& bt_nh)
  {
    ros::NodeHandle root_nh;
    publishers_ = std::make_unique<Pubs>();

    publishers_->map_sentry_data_pub_ = root_nh.advertise<rm_msgs::MapSentryData>("/map_sentry_data", 10);
    publishers_->marker_pub_ = root_nh.advertise<visualization_msgs::Marker>("/radar_marker", 1);
    publishers_->aim_priority_pub_ = bt_nh.advertise<rm_msgs::PriorityArray>(
      "/armor_processor/priority/priority_arr", 1);
    publishers_->sentry_state_pub_ = bt_nh.advertise<std_msgs::String>("/custom_info", 1);
    publishers_->sentry_cmd_pub_ = bt_nh.advertise<rm_msgs::SentryCmd>("/sentry_cmd", 1);
    publishers_->conduct_point_pub_ = bt_nh.advertise<geometry_msgs::PoseStamped>("/conduct_point_in_map", 1);
    publishers_->attacking_target_pub_ = bt_nh.advertise<rm_msgs::SentryAttackingTarget>(
      "/sentry_target_to_referee", 1);
    publishers_->manual_to_referee_pub_ = bt_nh.advertise<rm_msgs::ManualToReferee>("/manual_to_referee", 1);

    publish_msgs = std::make_unique<Msgs>();
  }

  Publisher::Pubs* Publisher::getPublishers()
  {
    return publishers_.get();
  }

  Publisher::Msgs* Publisher::getPublishMsgs()
  {
    return publish_msgs.get();
  }
}
