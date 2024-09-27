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

#include <sched.h>
#include <unistd.h>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/rt_single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::RTSingleThreadedExecutor;

RTSingleThreadedExecutor::RTSingleThreadedExecutor(const rclcpp::ExecutorOptions & options, 
                                              unsigned int priority)
: rclcpp::Executor(options) {
  priority_ = priority;
  if(priority < 1 || priority > 99){
    priority_ = 60;
    RCLCPP_WARN(
      rclcpp::get_logger("rclcpp"),
      "Scheduler priority must be a integer between 1 (lowest priority) and 99 (highest priority).\n"
      "Scheduler priority has been set to %d which is a safe value.", priority_);
  }
}

RTSingleThreadedExecutor::~RTSingleThreadedExecutor() {}

void
RTSingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;
    if (get_next_executable(any_executable)) {
      sched_param param;
      param.sched_priority = priority_;
      sched_setscheduler(getpid(), SCHED_FIFO, &param);
      execute_any_executable(any_executable);
    }
  }
}
