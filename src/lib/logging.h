#pragma once

#define QUILL_DISABLE_NON_PREFIXED_MACROS

#include <quill/Logger.h>
#include <quill/LogMacros.h>

void InitLogging();

quill::Logger* GetLogger();

#define LOG_TRACE_L3(fmt, ...) QUILL_LOG_TRACE_L3(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_TRACE_L2(fmt, ...) QUILL_LOG_TRACE_L2(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_TRACE_L1(fmt, ...) QUILL_LOG_TRACE_L1(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)    QUILL_LOG_DEBUG(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)     QUILL_LOG_INFO(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)  QUILL_LOG_WARNING(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)    QUILL_LOG_ERROR(::GetLogger(), fmt, ##__VA_ARGS__)
#define LOG_CRITICAL(fmt, ...) QUILL_LOG_CRITICAL(::GetLogger(), fmt, ##__VA_ARGS__)
