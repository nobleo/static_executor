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

#include "static_executor/executable_list.hpp"

using rclcpp::executor::ExecutableList;

ExecutableList::ExecutableList()
:
  number_of_subscription(0),
  number_of_timer(0),
  number_of_service(0),
  number_of_client(0)
  {}

ExecutableList::~ExecutableList()
{
  // Make sure that discarded (taken but not executed) AnyExecutable's have
  // their callback groups reset. This can happen when an executor is canceled
  // between taking an AnyExecutable and executing it.
  if (callback_group) {
    callback_group->can_be_taken_from().store(true);
  }
}
