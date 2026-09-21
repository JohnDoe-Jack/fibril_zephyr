/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <cstdint>

namespace armctl {

/* Backpressure retries consumed by the enable sequence. The CAN-specific
 * accounting view belongs to the integration layer; the controller only
 * reports how many retries it intentionally absorbed.
 */
class AbsorbedDrops {
public:
    void record(uint32_t n)
    {
        last_ = n;
        total_ += n;
    }

    uint32_t last() const { return last_; }
    uint64_t total() const { return total_; }

private:
    uint32_t last_ = 0;
    uint64_t total_ = 0;
};

} // namespace armctl
