#ifndef RCLCPP__EXECUTABLE_LIST_HPP_
#define RCLCPP__EXECUTABLE_LIST_HPP_

#include <memory>
#include <vector>

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

/// This struct contains subscriptionbase, timerbase, etc. which can be used to run callbacks.
struct ExecutableList
{
  RCLCPP_PUBLIC
  ExecutableList();

  RCLCPP_PUBLIC
  virtual ~ExecutableList();

  // Vector containing the SubscriptionBase of all the subscriptions added to the executor.
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscription;
  // Contains the count of added subscriptions
  size_t number_of_subscriptions;
  // Vector containing the TimerBase of all the timers added to the executor.
  std::vector<rclcpp::TimerBase::SharedPtr> timer;
  // Contains the count of added timers
  size_t number_of_timers;
  // Vector containing the ServiceBase of all the services added to the executor.
  std::vector<rclcpp::ServiceBase::SharedPtr> service;
  // Contains the count of added services
  size_t number_of_services;
  // Vector containing the ClientBase of all the clients added to the executor.
  std::vector<rclcpp::ClientBase::SharedPtr> client;
  // Contains the count of added clients
  size_t number_of_clients;
  // Vector containing all the waitables added to the executor.
  std::vector<rclcpp::Waitable::SharedPtr> waitable;
  // Contains the count of added waitables
  size_t number_of_waitables;
};

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTABLE_LIST_HPP_
