#pragma once

#include <memory>

#include <perception/wall_extractor_interface.hpp>

namespace perception::wall_extractor {

struct Session : WallExtractorSession<Session> {
    using WallExtractorSession::WallExtractorSession;
};

CallbackReturn on_configure(std::shared_ptr<Session> sn);

using WallExtractor = WallExtractorBase<Session, on_configure>;

} // namespace perception::wall_extractor
