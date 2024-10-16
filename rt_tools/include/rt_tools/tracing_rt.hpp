/*
THIS CODE WAS TAKEN FROM THE REPOSITORY:
https://github.com/ros-realtime/roscon-2023-realtime-workshop.git
*/

#ifndef RT_TOOLS_TRACING_RT_HPP_
#define RT_TOOLS_TRACING_RT_HPP_

#include "cactus_rt/tracing.h"

void StartTracing(const char* app_name, const char* filename);
void StopTracing();
void RegisterThreadTracer(std::shared_ptr<cactus_rt::tracing::ThreadTracer> thread_tracer);

#endif