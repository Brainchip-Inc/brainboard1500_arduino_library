#pragma once
#include <cstddef>
#include <map>

#include "engine/dma.h"
#include "infra/hardware_driver.h"

#include "memory_mgr.h"

namespace akida {

class ExternalMemoryMgr {
 public:
  explicit ExternalMemoryMgr(MemoryMgr* mgr, HardwareDriver* driver)
      : mem_mgr_(mgr), driver_(driver) {}

  using AllocId = const void*;
  // Track a local address that will also be on the device.
  // It copies data on device if they are not accessible from akida (if they are
  // not in HardwareDriver akida_visible_memory range)
  dma::addr track_and_put_on_device_if_required(AllocId id, size_t byte_size);

  // release (untrack) data from device (if data was copied on it, memory is
  // freed)
  void release(AllocId id);

  // get on device address from id
  dma::addr tracked(AllocId id) const;

  // Untrack all memory, freeing it if they were copied on device, to restore
  // initial state
  void reset();

 private:
  // memory manager
  MemoryMgr* mem_mgr_;
  // hardware driver
  HardwareDriver* driver_;
  // allocation ledger, a map of id:addresss
  std::map<AllocId, uint32_t> alloc_ledger_;
};

}  // namespace akida
