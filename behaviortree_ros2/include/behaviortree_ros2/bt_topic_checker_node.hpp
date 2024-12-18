// Copyright (c) 2023 Davide Faconti, Unmanned Life
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Copied from bt_topic_sub_node.hpp - TODO do the inheritence better - Greg

#pragma once

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/assumption_checker_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/ros_node_params.hpp"
#include <boost/signals2.hpp>

namespace BT
{


/**
 * @brief Abstract class to wrap a Topic subscriber.
 * Considering the example in the tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
 *
 * The corresponding wrapper would be:
 *
 * class SubscriberNode: RosTopicSubCheckerNode<std_msgs::msg::String>
 *
 * The name of the topic will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "topic_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template<class TopicT>
class RosTopicSubCheckerNode : public BT::AssumptionCheckerNode
{
 public:
  // Type definitions
  using Subscriber = typename rclcpp::Subscription<TopicT>;

 protected: 
  struct SubscriberInstance
  {
    void init(std::shared_ptr<rclcpp::Node> node, const std::string& topic_name, const rclcpp::QoS& qos_profile)
    {
      // create a callback group for this particular instance
      callback_group = 
        node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      callback_group_executor.add_callback_group(
        callback_group, node->get_node_base_interface());

      rclcpp::SubscriptionOptions option;
      option.callback_group = callback_group;

    // The callback will broadcast to all the instances of RosTopicSubCheckerNode<T>
      auto callback = [this](const std::shared_ptr<TopicT> msg) 
      {
        broadcaster(msg);
      };
      subscriber =  node->create_subscription<TopicT>(topic_name, qos_profile, callback, option);
    }

    std::shared_ptr<Subscriber> subscriber;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor;
    boost::signals2::signal<void (const std::shared_ptr<TopicT>)> broadcaster;


  };

  static std::mutex& registryMutex()
  {
    static std::mutex sub_mutex;
    return sub_mutex;
  }

  // contains the fully-qualified name of the node and the name of the topic
  static std::unordered_map<std::string, std::shared_ptr<SubscriberInstance>>& getRegistry()
  {
    static std::unordered_map<std::string, std::shared_ptr<SubscriberInstance>> subscribers_registry;
    return subscribers_registry;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<SubscriberInstance> sub_instance_ = nullptr;
  std::shared_ptr<TopicT> last_msg_;
  std::string topic_name_;
  boost::signals2::connection signal_connection_;
  std::string subscriber_key_;
  rclcpp::QoS qos_profile_;

  rclcpp::Logger logger()
  {
    return node_->get_logger();
  }

 public:

  /** You are not supposed to instantiate this class directly, the factory will do it.
   * To register this class into the factory, use:
   *
   *    RegisterRosAction<DerivedClasss>(factory, params)
   *
   * Note that if the external_action_client is not set, the constructor will build its own.
   * */
  explicit RosTopicSubCheckerNode(const std::string & instance_name,
                           const BT::NodeConfig& conf,
                           const RosNodeParams& params);

  virtual ~RosTopicSubCheckerNode() 
  {
    signal_connection_.disconnect();
    // remove the subscribers from the static registry when the ALL the
    // instances of RosTopicSubCheckerNode are destroyed (i.e., the tree is destroyed)
    if(sub_instance_)
    {
      sub_instance_.reset();
      std::unique_lock lk(registryMutex());
      auto& registry = getRegistry();
      auto it = registry.find(subscriber_key_);
      // when the reference count is 1, means that the only instance is owned by the
      // registry itself. Delete it
      if(it != registry.end() && it->second.use_count() <= 1)
      {
        registry.erase(it);
        RCLCPP_INFO(logger(), "Remove subscriber [%s]", topic_name_.c_str() );
      }
    }
  }

  /**
   * @brief Any subclass of RosTopicNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = {
      InputPort<std::string>("topic_name", "__default__placeholder__", "Topic name")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  NodeStatus tick() override final;

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received, since the last tick.
   *                  Will be empty if no new message received.
   * 
   * @return the new status of the Node, based on last_msg
   */
  virtual NodeStatus onTick(const std::shared_ptr<TopicT>& last_msg) = 0;

private:

  bool createSubscriber(const std::string& topic_name, const rclcpp::QoS& qos_profile);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template<class T> inline
  RosTopicSubCheckerNode<T>::RosTopicSubCheckerNode(const std::string & instance_name,
                                      const NodeConfig &conf,
                                      const RosNodeParams& params)
    : BT::AssumptionCheckerNode(instance_name, conf),
      node_(params.nh), qos_profile_(params.qos_profile)
{ 
  // check port remapping
  auto portIt = config().input_ports.find("topic_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_topic_name = portIt->second;

    if(bb_topic_name.empty() || bb_topic_name == "__default__placeholder__")
    {
      if(params.default_port_value.empty()) {
        throw std::logic_error(
          "Both [topic_name] in the InputPort and the RosNodeParams are empty.");
      }
      else {
        createSubscriber(params.default_port_value, params.qos_profile);
      }
    }
    else if(!isBlackboardPointer(bb_topic_name))
    {
      // If the content of the port "topic_name" is not
      // a pointer to the blackboard, but a static string, we can
      // create the client in the constructor.
      createSubscriber(bb_topic_name, params.qos_profile);
    }
    else {
      // do nothing
      // createSubscriber will be invoked in the first tick().
    }
  }
  else {
    if(params.default_port_value.empty()) {
      throw std::logic_error(
        "Both [topic_name] in the InputPort and the RosNodeParams are empty.");
    }
    else {
      createSubscriber(params.default_port_value, params.qos_profile);
    }
  }
}

template<class T> inline
  bool RosTopicSubCheckerNode<T>::createSubscriber(const std::string& topic_name, const rclcpp::QoS& qos_profile)
{
  if(topic_name.empty())
  {
    throw RuntimeError("topic_name is empty");
  }
  if(sub_instance_)
  {
    throw RuntimeError("Can't call createSubscriber more than once");
  }

  // find SubscriberInstance in the registry
  std::unique_lock lk(registryMutex());
  subscriber_key_ = std::string(node_->get_fully_qualified_name()) + "/" + topic_name;

  auto& registry = getRegistry();
  auto it = registry.find(subscriber_key_);
  if(it == registry.end())
  {
    it = registry.insert( {subscriber_key_, std::make_shared<SubscriberInstance>() }).first;
    sub_instance_ = it->second;
    sub_instance_->init(node_, topic_name, qos_profile);

    RCLCPP_INFO(logger(), 
      "Node [%s] created Subscriber to topic [%s]",
      name().c_str(), topic_name.c_str() );
  }
  else {
    sub_instance_ = it->second;
  }


  // add "this" as received of the broadcaster
  signal_connection_ = sub_instance_->broadcaster.connect(
    [this](const std::shared_ptr<T> msg) { last_msg_ = msg; } );

  topic_name_ = topic_name;

  return true;
}


template<class T> inline
  NodeStatus RosTopicSubCheckerNode<T>::tick()
{
  // First, check if the subscriber_ is valid and that the name of the
  // topic_name in the port didn't change.
  // otherwise, create a new subscriber
  if(!sub_instance_)
  {
    std::string topic_name;
    getInput("topic_name", topic_name);
    createSubscriber(topic_name, qos_profile_); 
  }

  auto CheckStatus = [](NodeStatus status)
  {
    if( !isStatusCompleted(status) )
    {
      throw std::logic_error("RosTopicSubCheckerNode: the callback must return" 
                             "either SUCCESS or FAILURE");
    }
    return status;
  };
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  sub_instance_->callback_group_executor.spin_some();
  auto status = CheckStatus (onTick(last_msg_));
  last_msg_ = nullptr;

  return status;
}

}  // namespace BT

