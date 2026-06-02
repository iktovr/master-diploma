#include "logging.h"

#include <mutex>

#include <quill/Backend.h>
#include <quill/Frontend.h>
#include <quill/sinks/ConsoleSink.h>

namespace {

constexpr const char* kRootLoggerName = "root";
constexpr const char* kRootSinkId = "root_console_sink";

std::once_flag g_init_flag;
quill::Logger* g_root_logger = nullptr;

void DoInit() {
    quill::Backend::start();
    g_root_logger = quill::Frontend::create_or_get_logger(
        kRootLoggerName,
        quill::Frontend::create_or_get_sink<quill::ConsoleSink>(kRootSinkId));
}

}  // namespace

void InitLogging() {
    std::call_once(g_init_flag, DoInit);
}

quill::Logger* GetLogger() {
    InitLogging();
    return g_root_logger;
}
