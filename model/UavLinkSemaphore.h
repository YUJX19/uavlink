#ifndef UAV_LINK_SEMAPHORE_H
#define UAV_LINK_SEMAPHORE_H

#include <atomic>
#include <cstdint>

namespace ns3 {
namespace uavlink {

/**
 * \brief Semaphore implementation for UAVLink module in NS3.
 */
class UavLinkSemaphore
{
public:
    explicit UavLinkSemaphore() = default;

    static inline uint8_t atomic_read8(const std::atomic<uint8_t>& mem)
    {
        return mem.load(std::memory_order_acquire);
    }

    static inline uint8_t atomic_cas8(std::atomic<uint8_t>& mem, uint8_t with, uint8_t cmp)
    {
        uint8_t expected = cmp;
        mem.compare_exchange_strong(expected, with, std::memory_order_acq_rel);
        return expected;
    }

    static inline uint8_t atomic_add8(std::atomic<uint8_t>& mem, uint8_t val)
    {
        return mem.fetch_add(val, std::memory_order_acq_rel);
    }

    static inline bool atomic_add_unless8(std::atomic<uint8_t>& mem, uint8_t value, uint8_t unless_this)
    {
        uint8_t old_val = mem.load(std::memory_order_acquire);
        while (old_val != unless_this)
        {
            if (mem.compare_exchange_weak(old_val, old_val + value, std::memory_order_acq_rel))
            {
                return true;
            }
        }
        return false;
    }

    static inline bool sem_try_wait(std::atomic<uint8_t>& mem)
    {
        return atomic_add_unless8(mem, static_cast<uint8_t>(-1), 0);
    }

    static inline void sem_wait(std::atomic<uint8_t>& mem)
    {
        while (!sem_try_wait(mem))
        {
            // Optionally, add a delay or yield thread to avoid busy-waiting
        }
    }

    static inline uint8_t sem_post(std::atomic<uint8_t>& mem)
    {
        return atomic_add8(mem, 1);
    }
};

} // namespace uavlink
} // namespace ns3

#endif // UAV_LINK_SEMAPHORE_H
