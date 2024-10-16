/*
THIS CODE WAS TAKEN FROM THE REPOSITORY:
https://github.com/ros-realtime/roscon-2023-realtime-workshop.git
*/
#include "rt_tools/tracing_rt.hpp"

using cactus_rt::tracing::FileSink;
using cactus_rt::tracing::ThreadTracer;
using cactus_rt::tracing::TraceAggregator;

std::unique_ptr<TraceAggregator> trace_aggregator;

void StartTracing(const char* app_name, const char* filename) {
    // Enable the tracing.
    cactus_rt::tracing::EnableTracing();

    // Create the trace aggregator that will pop the queues and write the events to sinks.
    trace_aggregator = std::make_unique<TraceAggregator>(app_name);

    // Create the file sink so the data aggregated by the TraceAggregator will be written to somewhere.
    auto file_sink = std::make_shared<FileSink>(filename);
    trace_aggregator->RegisterSink(file_sink);

    quill::start();
    trace_aggregator->Start();
}

void StopTracing() {
    cactus_rt::tracing::DisableTracing();

    trace_aggregator->RequestStop();
    trace_aggregator->Join();
    trace_aggregator = nullptr;  // Destroy the trace aggregator and free all resources.
}

void RegisterThreadTracer(std::shared_ptr<ThreadTracer> thread_tracer) {
    trace_aggregator->RegisterThreadTracer(thread_tracer);
}