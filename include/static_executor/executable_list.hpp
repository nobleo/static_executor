// Copyright 2019 Nobleo Technology.
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

#ifndef RCLCPP__EXECUTABLE_LIST_HPP_
#define RCLCPP__EXECUTABLE_LIST_HPP_

#include <memory>

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executor
{

struct ExecutableList
{
  RCLCPP_PUBLIC
  ExecutableList();

  RCLCPP_PUBLIC
  virtual ~ExecutableList();

  // More than one of the following pointers can be set.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscription;
  size_t number_of_subscription;
  std::vector<rclcpp::TimerBase::SharedPtr> timer;
  size_t number_of_timer;
  std::vector<rclcpp::ServiceBase::SharedPtr> service;
  size_t number_of_service;
  std::vector<rclcpp::ClientBase::SharedPtr> client;
  size_t number_of_client;
  std::vector<rclcpp::Waitable::SharedPtr> waitable;
  size_t number_of_waitable;
  // These are used to keep the scope on the containing items
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base;
};

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTABLE_LIST_HPP_
