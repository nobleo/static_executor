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

#include "static_executor/executors/static_single_threaded_executor.hpp"
#include "static_executor/executable_list.hpp"
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::StaticSingleThreadedExecutor;
using rclcpp::executor::ExecutableList;

StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(const rclcpp::executor::ExecutorArgs & args)
: executor::StaticExecutor(args) {}

StaticSingleThreadedExecutor::~StaticSingleThreadedExecutor() {}


void
StaticSingleThreadedExecutor::spin()
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