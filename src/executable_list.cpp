#include "static_executor/executable_list.hpp"

using rclcpp::executor::ExecutableList;

ExecutableList::ExecutableList()
: number_of_subscriptions(0),
  number_of_timers(0),
  number_of_services(0),
  number_of_clients(0),
  number_of_waitables(0)
{}

ExecutableList::~ExecutableList()
{}
