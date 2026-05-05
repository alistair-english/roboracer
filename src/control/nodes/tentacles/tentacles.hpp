#pragma once

#include <memory>

#include <control/tentacles_interface.hpp>

namespace control::tentacles {

struct Session : TentaclesSession<Session> {
    using TentaclesSession::TentaclesSession;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);

using Tentacles = TentaclesBase<Session, on_configure>;

} // namespace control::tentacles
