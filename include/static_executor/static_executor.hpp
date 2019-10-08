// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__STATIC_EXECUTOR_HPP_
#define RCLCPP__STATIC_EXECUTOR_HPP_

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "rcl/guard_condition.h"
#include "rcl/wait.h"

#include "rclcpp/executor.hpp"
#include "static_executor/executable_list.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

// Forward declaration is used in convenience method signature.
class Node;

namespace executor
{

/// Return codes to be used with spin_until_future_complete.
/**
 * SUCCESS: The future is complete and can be accessed with "get" without blocking.
 * INTERRUPTED: The future is not complete, spinning was interrupted by Ctrl-C or another error.
 * TIMEOUT: Spinning timed out.
 */

RCLCPP_PUBLIC
std::ostream &
operator<<(std::ostream & os, const FutureReturnCode & future_return_code);

RCLCPP_PUBLIC
std::string
to_string(const FutureReturnCode & future_return_code);

///
/**
 * Options to be passed to the executor constructor.
 */

/// Coordinate the order and timing of available communication tasks.
/**
 * Executor provides spin functions (including spin_node_once and spin_some).
 * It coordinates the nodes and callback groups by looking for available work and completing it,
 * based on the threading or concurrency scheme provided by the subclass implementation.
 * An example of available work is executing a subscription callback, or a timer callback.
 * The executor structure allows for a decoupling of the communication graph and the execution
 * model.
 * See SingleThreadedExecutor and MultiThreadedExecutor for examples of execution paradigms.
 */
class StaticExecutor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(StaticExecutor)

  /// Default constructor.
  // \param[in] ms The memory strategy to be used with this executor.
  RCLCPP_PUBLIC
  explicit StaticExecutor(const ExecutorArgs & args = ExecutorArgs());

  /// Default destructor.
  RCLCPP_PUBLIC
  virtual ~StaticExecutor();

  /// Do work periodically as it becomes available to us. Blocking call, may block indefinitely.
  virtual void
  spin() = 0;

  /// Add a node to the executor.
  /**
   * An executor can have zero or more nodes which provide work during `spin` functions.
   * \param[in] node_ptr Shared pointer to the node to be added.
   * \param[in] notify True to trigger the interrupt guard condition during this function. If
   * the executor is blocked at the rmw layer while waiting for work and it is notified that a new
   * node was added, it will wake up.
   */
  RCLCPP_PUBLIC
  virtual void
  add_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Remove a node from the executor.
  /**
   * \param[in] node_ptr Shared pointer to the node to remove.
   * \param[in] notify True to trigger the interrupt guard condition and wake up the executor.
   * This is useful if the last node was removed from the executor while the executor was blocked
   * waiting for work in another thread, because otherwise the executor would never be notified.
   */
  RCLCPP_PUBLIC
  virtual void
  remove_node(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify = true);

  /// Convenience function which takes Node and forwards NodeBaseInterface.
  RCLCPP_PUBLIC
  virtual void
  remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify = true);

  /// Complete all available queued work without blocking.
  /**
   * This function can be overridden. The default implementation is suitable for a
   * single-threaded model of execution.
   * Adding subscriptions, timers, services, etc. with blocking callbacks will cause this function
   * to block (which may have unintended consequences).
   *
   * \param[in] max_duration The maximum amount of time to spend executing work, or 0 for no limit.
   * Note that spin_some() may take longer than this time as it only returns once max_duration has
   * been exceeded.
   */

  /// Spin (blocking) until the future is complete, it times out waiting, or rclcpp is interrupted.
  /**
   * \param[in] future The future to wait on. If SUCCESS, the future is safe to access after this
   *   function.
   * \param[in] timeout Optional timeout parameter, which gets passed to Executor::spin_node_once.
   *   `-1` is block forever, `0` is non-blocking.
   *   If the time spent inside the blocking loop exceeds this timeout, return a TIMEOUT return
   *   code.
   * \return The return code, one of `SUCCESS`, `INTERRUPTED`, or `TIMEOUT`.
   */
  template<typename ResponseT, typename TimeRepT = int64_t, typename TimeT = std::milli>
  rclcpp::executor::FutureReturnCode
  spin_until_future_complete(
    std::shared_future<ResponseT> & future,
    std::chrono::duration<TimeRepT, TimeT> timeout = std::chrono::duration<TimeRepT, TimeT>(-1))
  {
    std::future_status status = future.wait_for(std::chrono::seconds(0));
    if (status == std::future_status::ready) {
      return rclcpp::executor::FutureReturnCode::SUCCESS;
    }

    auto end_time = std::chrono::steady_clock::now();
    std::chrono::nanoseconds timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      timeout);
    if (timeout_ns > std::chrono::nanoseconds::zero()) {
      end_time += timeout_ns;
    }
    std::chrono::nanoseconds timeout_left = timeout_ns;

    rclcpp::executor::ExecutableList executable_list;
    run_collect_entities();
    get_executable_list(executable_list);
    while (rclcpp::ok(this->context_)) {
      execute_wait_set(executable_list, timeout_left);
      // Check if the future is set, return SUCCESS if it is.
      status = future.wait_for(std::chrono::seconds(0));
      if (status == std::future_status::ready) {
        return rclcpp::executor::FutureReturnCode::SUCCESS;
      }
      // If the original timeout is < 0, then this is blocking, never TIMEOUT.
      if (timeout_ns < std::chrono::nanoseconds::zero()) {
        continue;
      }
      // Otherwise check if we still have time to wait, return TIMEOUT if not.
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return rclcpp::executor::FutureReturnCode::TIMEOUT;
      }
      // Subtract the elapsed time from the original timeout.
      timeout_left = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - now);
    }

    // The future did not complete before ok() returned false, return INTERRUPTED.
    return rclcpp::executor::FutureReturnCode::INTERRUPTED;
  }

  /// Cancel any running spin* function, causing it to return.
  /* This function can be called asynchonously from any thread. */
  RCLCPP_PUBLIC
  void
  cancel();

  /// Support dynamic switching of the memory strategy.
  /**
   * Switching the memory strategy while the executor is spinning in another threading could have
   * unintended consequences.
   * \param[in] memory_strategy Shared pointer to the memory strategy to set.
   */
  RCLCPP_PUBLIC
  void
  set_memory_strategy(memory_strategy::MemoryStrategy::SharedPtr memory_strategy);

protected:
  RCLCPP_PUBLIC
  void
  spin_node_once_nanoseconds(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
    std::chrono::nanoseconds timeout);

  // StaticExecutor functions
  RCLCPP_PUBLIC
  void
  execute_wait_set(executor::ExecutableList & exec_list,
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void
  get_timer_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_subscription_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_service_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_client_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_waitable_list(executor::ExecutableList & exec_list);

  RCLCPP_PUBLIC
  void
  get_executable_list(executor::ExecutableList & executable_list,
  std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void run_collect_entities();

  RCLCPP_PUBLIC
  void refresh_wait_set(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  void prepare_wait_set();

  /// Find the next available executable and do the work associated with it.
  /** \param[in] any_exec Union structure that can hold any executable type (timer, subscription,
   * service, client).
   */

  RCLCPP_PUBLIC
  static void
  execute_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_intra_process_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription);

  RCLCPP_PUBLIC
  static void
  execute_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  static void
  execute_service(rclcpp::ServiceBase::SharedPtr service);

  RCLCPP_PUBLIC
  static void
  execute_client(rclcpp::ClientBase::SharedPtr client);

  RCLCPP_PUBLIC
  void
  wait_for_work(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group);

  RCLCPP_PUBLIC
  rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_timer(rclcpp::TimerBase::SharedPtr timer);

  RCLCPP_PUBLIC
  void
  get_next_timer(AnyExecutable & any_exec);

  /// Spinning state, used to prevent multi threaded calls to spin and to cancel blocking spins.
  std::atomic_bool spinning;

  /// Guard condition for signaling the rmw layer to wake up for special events.
  rcl_guard_condition_t interrupt_guard_condition_ = rcl_get_zero_initialized_guard_condition();

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t wait_set_ = rcl_get_zero_initialized_wait_set();

  /// The memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  /// The context associated with this executor.
  std::shared_ptr<rclcpp::Context> context_;

protected:
  RCLCPP_DISABLE_COPY(StaticExecutor)

  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;
  std::list<const rcl_guard_condition_t *> guard_conditions_;
  size_t old_number_of_guard_conditions_;
};

}  // namespace executor
}  // namespace rclcpp

#endif  // RCLCPP__STATIC_EXECUTOR_HPP_