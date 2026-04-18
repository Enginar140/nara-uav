#pragma once

#include <mavlink_types.h>
#include <common/mavlink.h>

#include <cstdint>

namespace uav::comm {

struct MAVLinkMessage {
    mavlink_message_t raw{};
    uint64_t received_at_us = 0;

    uint8_t msgid() const { return raw.msgid; }
    uint8_t sysid() const { return raw.sysid; }
    uint8_t compid() const { return raw.compid; }
};

} // namespace uav::comm
