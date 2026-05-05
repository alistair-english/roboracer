#pragma once

#include <memory>

#include <perception/midline_tracer_interface.hpp>

namespace perception::midline_tracer {

struct Session : MidlineTracerSession<Session> {
    using MidlineTracerSession::MidlineTracerSession;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);

using MidlineTracer = MidlineTracerBase<Session, on_configure>;

} // namespace perception::midline_tracer
