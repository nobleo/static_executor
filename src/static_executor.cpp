// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <memory>
#include <string>
#include <type_traits>

#include "rcl/allocator.h"
#include "rcl/error_handling.h"

#include "rclcpp/exceptions.hpp"
#include "static_executor/static_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/utilities.hpp"

#include "rcl_interfaces/msg/intra_process_message.hpp"

#include "rcutils/logging_macros.h"

using rclcpp::exceptions::throw_from_rcl_error;
using rclcpp::executor::AnyExecutable;
using rclcpp::executor::StaticExecutor;
using rclcpp::executor::ExecutorArgs;
using rclcpp::executor::FutureReturnCode;

StaticExecutor::StaticExecutor(const ExecutorArgs & args)
: spinning(false),
  memory_strategy_(args.memory_strategy)
{
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(
    &interrupt_guard_condition_, args.context->get_rcl_context().get(), guard_condition_options);
  if (RCL_RET_OK != ret) {
    throw_from_rcl_error(ret, "Failed to create interrupt guard condition in Executor constructor");
  }

  // The number of guard conditions is always at least 2: 1 for the ctrl-c guard cond,
  // and one for the executor's guard cond (interrupt_guard_condition_)

  // Put the global ctrl-c guard condition in
  memory_strategy_->add_guard_condition(args.context->get_interrupt_guard_condition(&wait_set_));

  // Put the executor's guard condition in
  memory_strategy_->add_guard_condition(&interrupt_guard_condition_);
  rcl_allocator_t allocator = memory_strategy_->get_allocator();

  // Store the context for later use.
  context_ = args.context;

  ret = rcl_wait_set_init(
    &wait_set_,
    0, 2, 0, 0, 0, 0,
    context_->get_rcl_context().get(),
    allocator);
  if (RCL_RET_OK != ret) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to create wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
    if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "failed to destroy guard condition: %s", rcl_get_error_string().str);
      rcl_reset_error();
    }
    throw std::runtime_error("Failed to create wait set in Executor constructor");
  }
}

StaticExecutor::~StaticExecutor()
{
  // Disassocate all nodes
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node) {
      std::atomic_bool & has_executor = node->get_associated_with_executor_atomic();
      has_executor.store(false);
    }
  }
  weak_nodes_.clear();
  for (auto & guard_condition : guard_conditions_) {
    memory_strategy_->remove_guard_condition(guard_condition);
  }
  guard_conditions_.clear();

  // Finalize the wait set.
  if (rcl_wait_set_fini(&wait_set_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to destroy wait set: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  // Finalize the interrupt guard condition.
  if (rcl_guard_condition_fini(&interrupt_guard_condition_) != RCL_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "failed to destroy guard condition: %s", rcl_get_error_string().str);
    rcl_reset_error();
  }
  // Remove and release the sigint guard condition
  memory_strategy_->remove_guard_condition(context_->get_interrupt_guard_condition(&wait_set_));
  context_->release_interrupt_guard_condition(&wait_set_, std::nothrow);
}

void
StaticExecutor::add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // If the node already has an executor
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  if (has_executor.exchange(true)) {
    throw std::runtime_error("Node has already been added to an executor.");
  }
  // Check to ensure node not already added
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (node == node_ptr) {
      // TODO(jacquelinekay): Use a different error here?
      throw std::runtime_error("Cannot add node to executor, node already added.");
    }
  }
  weak_nodes_.push_back(node_ptr);
  guard_conditions_.push_back(node_ptr->get_notify_guard_condition());
  if (notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
  // Add the node's notify condition to the guard condition handles
  memory_strategy_->add_guard_condition(node_ptr->get_notify_guard_condition());
}

void
StaticExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
StaticExecutor::remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool node_removed = false;
  {
    auto node_it = weak_nodes_.begin();
    auto gc_it = guard_conditions_.begin();
    while (node_it != weak_nodes_.end()) {
      bool matched = (node_it->lock() == node_ptr);
      if (matched) {
        node_it = weak_nodes_.erase(node_it);
        gc_it = guard_conditions_.erase(gc_it);
        node_removed = true;
      } else {
        ++node_it;
        ++gc_it;
      }
    }
  }
  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
  if (notify) {
    // If the node was matched and removed, interrupt waiting
    if (node_removed) {
      if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
        throw std::runtime_error(rcl_get_error_string().str);
      }
    }
  }
  memory_strategy_->remove_guard_condition(node_ptr->get_notify_guard_condition());
}

void
StaticExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
StaticExecutor::cancel()
{
  spinning.store(false);
  if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
    throw std::runtime_error(rcl_get_error_string().str);
  }
}

void
StaticExecutor::set_memory_strategy(rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy)
{
  if (memory_strategy == nullptr) {
    throw std::runtime_error("Received NULL memory strategy in executor.");
  }
  memory_strategy_ = memory_strategy;
}

void
StaticExecutor::execute_subscription(
  rclcpp::SubscriptionBase::SharedPtr subscription)
{
  rmw_message_info_t message_info;
  message_info.from_intra_process = false;

  if (subscription->is_serialized()) {
    auto serialized_msg = subscription->create_serialized_message();
    auto ret = rcl_take_serialized_message(
      subscription->get_subscription_handle().get(),
      serialized_msg.get(), &message_info, nullptr);
    if (RCL_RET_OK == ret) {
      auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
      subscription->handle_message(void_serialized_msg, message_info);
    } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "take_serialized failed for subscription on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
    subscription->return_serialized_message(serialized_msg);
  } else if (subscription->can_loan_messages()) {
    void * loaned_msg = nullptr;
    auto ret = rcl_take_loaned_message(
      subscription->get_subscription_handle().get(),
      &loaned_msg,
      &message_info,
      nullptr);
    if (RCL_RET_OK == ret) {
      subscription->handle_loaned_message(loaned_msg, message_info);
    } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "take_loaned failed for subscription on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
    ret = rcl_return_loaned_message_from_subscription(
      subscription->get_subscription_handle().get(),
      loaned_msg);
    if (RCL_RET_OK != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "return_loaned_message failed for subscription on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
    }
    loaned_msg = nullptr;
  } else {
    std::shared_ptr<void> message = subscription->create_message();
    auto ret = rcl_take(
      subscription->get_subscription_handle().get(),
      message.get(), &message_info, nullptr);
    if (RCL_RET_OK == ret) {
      subscription->handle_message(message, message_info);
    } else if (RCL_RET_SUBSCRIPTION_TAKE_FAILED != ret) {
      RCUTILS_LOG_ERROR_NAMED(
        "rclcpp",
        "could not deserialize serialized message on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
      rcl_reset_error();
    }
    subscription->return_message(message);
  }
}

void
StaticExecutor::execute_timer(
  rclcpp::TimerBase::SharedPtr timer)
{
  timer->execute_callback();
}

void
StaticExecutor::execute_service(
  rclcpp::ServiceBase::SharedPtr service)
{
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  rcl_ret_t status = rcl_take_request(
    service->get_service_handle().get(),
    request_header.get(),
    request.get());
  if (status == RCL_RET_OK) {
    service->handle_request(request_header, request);
  } else if (status != RCL_RET_SERVICE_TAKE_FAILED) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "take request failed for server of service '%s': %s",
      service->get_service_name(), rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void
StaticExecutor::execute_client(
  rclcpp::ClientBase::SharedPtr client)
{
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  rcl_ret_t status = rcl_take_response(
    client->get_client_handle().get(),
    request_header.get(),
    response.get());
  if (status == RCL_RET_OK) {
    client->handle_response(request_header, response);
  } else if (status != RCL_RET_CLIENT_TAKE_FAILED) {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "take response failed for client of service '%s': %s",
      client->get_service_name(), rcl_get_error_string().str);
    rcl_reset_error();
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
StaticExecutor::get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group)
{
  if (!group) {
    return nullptr;
  }
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto callback_group = weak_group.lock();
      if (callback_group == group) {
        return node;
      }
    }
  }
  return nullptr;
}

rclcpp::callback_group::CallbackGroup::SharedPtr
StaticExecutor::get_group_by_timer(rclcpp::TimerBase::SharedPtr timer)
{
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group) {
        continue;
      }
      auto timer_ref = group->find_timer_ptrs_if(
        [timer](const rclcpp::TimerBase::SharedPtr & timer_ptr) -> bool {
          return timer_ptr == timer;
        });
      if (timer_ref) {
        return group;
      }
    }
  }
  return rclcpp::callback_group::CallbackGroup::SharedPtr();
}

void
StaticExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  rclcpp::executor::ExecutableList executable_list;
  run_collect_entities();
  get_executable_list(executable_list);

  while (rclcpp::ok(this->context_) && spinning.load()) {
    execute_wait_set(executable_list);
  }
}

void
StaticExecutor::get_timer_list(ExecutableList & exec_list)
{
  // Clear the previous timers (if any) from the ExecutableList struct
  exec_list.timer.clear();
  exec_list.number_of_timers = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      group->find_timer_ptrs_if([&exec_list](const rclcpp::TimerBase::SharedPtr & timer) {
          if(timer){
            // If any timer is found, push it in the exec_list struct
            exec_list.timer.push_back(timer);
            exec_list.number_of_timers++;
          }
          return false;
        });
    }
  }
}

void
StaticExecutor::get_subscription_list(ExecutableList & exec_list)
{
  // Clear the previous subscriptions (if any) from the ExecutableList struct
  exec_list.subscription.clear();
  exec_list.number_of_subscriptions = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      group->find_subscription_ptrs_if([&exec_list](
        const rclcpp::SubscriptionBase::SharedPtr & subscription) {
          if(subscription){
            // If any subscription (intra-process as well) is found, push it in the exec_list struct
            exec_list.subscription.push_back(subscription);
            exec_list.number_of_subscriptions++;
          }
          return false;
        });
    }
  }
}

void
StaticExecutor::get_service_list(ExecutableList & exec_list)
{
  // Clear the previous services (if any) from the ExecutableList struct
  exec_list.service.clear();
  exec_list.number_of_services = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      group->find_service_ptrs_if([&exec_list](const rclcpp::ServiceBase::SharedPtr & service) {
          if(service){
            // If any service is found, push it in the exec_list struct
            exec_list.service.push_back(service);
            exec_list.number_of_services++;
          }
          return false;
        });
    }
  }
}

void
StaticExecutor::get_client_list(ExecutableList & exec_list)
{
  // Clear the previous clients (if any) from the ExecutableList struct
  exec_list.client.clear();
  exec_list.number_of_clients = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      group->find_client_ptrs_if([&exec_list](const rclcpp::ClientBase::SharedPtr & client) {
          if(client){
            // If any client is found, push it in the exec_list struct
            exec_list.client.push_back(client);
            exec_list.number_of_clients++;
          }
          return false;
        });
    }
  }
}

void
StaticExecutor::get_waitable_list(ExecutableList & exec_list)
{
  // Clear the previous waitables (if any) from the ExecutableList struct
  exec_list.waitable.clear();
  exec_list.number_of_waitables = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();

      group->find_waitable_ptrs_if([&exec_list](const rclcpp::Waitable::SharedPtr & waitable) {
          if(waitable){
            // If any waitable is found, push it in the exec_list struct
            exec_list.waitable.push_back(waitable);
            exec_list.number_of_waitables++;
          }
            return false;
        });
    }
  }
}

void
StaticExecutor::get_executable_list(
  ExecutableList & executable_list)
{
  // prepare the wait_set
  prepare_wait_set();

  get_timer_list(executable_list);
  get_subscription_list(executable_list);
  get_service_list(executable_list);
  get_client_list(executable_list);
  get_waitable_list(executable_list);
}

// Function to run the callbacks from wait_set directly
void
StaticExecutor::execute_wait_set(
  ExecutableList & exec_list, std::chrono::nanoseconds timeout)
{
 refresh_wait_set(timeout);
  // Execute all the ready subscriptions
  for (size_t i = 0; i < wait_set_.size_of_subscriptions; ++i) {
    if (wait_set_.size_of_subscriptions && i < exec_list.number_of_subscriptions) {
      if (wait_set_.subscriptions[i]) {
        execute_subscription(exec_list.subscription[i]);
      }
    }
  }
  // Execute all the ready timers
  for (size_t i = 0; i < wait_set_.size_of_timers; ++i) {
    if (wait_set_.size_of_timers && i < exec_list.number_of_timers) {
      if (wait_set_.timers[i] && exec_list.timer[i]->is_ready()) {
        execute_timer(exec_list.timer[i]);
      }
    }
  }
  // Execute all the ready services
  for (size_t i = 0; i < wait_set_.size_of_services; ++i) {
    if (wait_set_.size_of_services && i < exec_list.number_of_services) {
      if (wait_set_.services[i]) {
        execute_service(exec_list.service[i]);
      }
    }
  }
  // Execute all the ready clients
  for (size_t i = 0; i < wait_set_.size_of_clients; ++i) {
    if (wait_set_.size_of_clients && i < exec_list.number_of_clients) {
      if (wait_set_.clients[i]) {
        execute_client(exec_list.client[i]);
      }
    }
  }
  // Execute all the ready waitables
  for (size_t i = 0; i < exec_list.number_of_waitables; ++i) {
    if (exec_list.number_of_waitables && exec_list.waitable[i]->is_ready(&wait_set_)) {
      exec_list.waitable[i]->execute();
    }
  }
  // Check the guard_conditions to see if anything is added to the executor
  for (size_t i = 0; i < wait_set_.size_of_guard_conditions; ++i) {
    if (wait_set_.guard_conditions[i] &&
        wait_set_.guard_conditions[i]!= context_->get_interrupt_guard_condition(&wait_set_) &&
        wait_set_.guard_conditions[i]!= &interrupt_guard_condition_) {
      run_collect_entities();
      get_executable_list(exec_list);
      break;
    }
  }
}

void
StaticExecutor::run_collect_entities()
{
  memory_strategy_->clear_handles();
  bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    auto node_it = weak_nodes_.begin();
    auto gc_it = guard_conditions_.begin();
    while (node_it != weak_nodes_.end()) {
      if (node_it->expired()) {
        node_it = weak_nodes_.erase(node_it);
        memory_strategy_->remove_guard_condition(*gc_it);
        gc_it = guard_conditions_.erase(gc_it);
      } else {
        ++node_it;
        ++gc_it;
      }
    }
  }
}

void
StaticExecutor::prepare_wait_set()
{
  // Collect the subscriptions and timers to be waited on

  // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }
  // The size of waitables are accounted for in size of the other entities
  rcl_ret_t ret = rcl_wait_set_resize(
    &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
    memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
    memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
    memory_strategy_->number_of_ready_events());
  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
            std::string("Couldn't resize the wait set : ") + rcl_get_error_string().str);
  }
}

void
StaticExecutor::refresh_wait_set(std::chrono::nanoseconds timeout)
{
    // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  if (!memory_strategy_->add_handles_to_wait_set(&wait_set_)) {
    throw std::runtime_error("Couldn't fill wait set");
  }
  rcl_ret_t status =
    rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }
}

std::ostream &
rclcpp::executor::operator<<(std::ostream & os, const FutureReturnCode & future_return_code)
{
  return os << to_string(future_return_code);
}

std::string
rclcpp::executor::to_string(const FutureReturnCode & future_return_code)
{
  using enum_type = std::underlying_type<FutureReturnCode>::type;
  std::string prefix = "Unknown enum value (";
  std::string ret_as_string = std::to_string(static_cast<enum_type>(future_return_code));
  switch (future_return_code) {
    case FutureReturnCode::SUCCESS:
      prefix = "SUCCESS (";
      break;
    case FutureReturnCode::INTERRUPTED:
      prefix = "INTERRUPTED (";
      break;
    case FutureReturnCode::TIMEOUT:
      prefix = "TIMEOUT (";
      break;
  }
  return prefix + ret_as_string + ")";
}
