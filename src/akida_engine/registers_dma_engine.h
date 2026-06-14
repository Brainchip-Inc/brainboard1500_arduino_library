#pragma once

#include <cstdint>
#include "infra/registers_common.h"

namespace akida {

// DMA controllers offsets, to be applied after top level register base
static constexpr uint32_t DMA_EVENT_REG_BASE = 0x00020000;
static constexpr uint32_t DMA_HRC_REG_BASE = 0x00028000;
static constexpr uint32_t DMA_CONFIG_REG_BASE = 0x00030000;
static inline uint32_t dma_event_reg_base(const uint32_t top_level_reg_base) {
  return top_level_reg_base + DMA_EVENT_REG_BASE;
}
static inline uint32_t dma_hrc_reg_base(const uint32_t top_level_reg_base) {
  return top_level_reg_base + DMA_HRC_REG_BASE;
}
static inline uint32_t dma_config_reg_base(const uint32_t top_level_reg_base) {
  return top_level_reg_base + DMA_CONFIG_REG_BASE;
}

// DMA core register
static constexpr uint32_t DMA_CTRL_REG = 0x0;
static constexpr RegDetail DMA_CTRL_VERSION(0, 2);
static constexpr RegDetail DMA_CTRL_RUN(8);
static constexpr RegDetail DMA_CTRL_SOFT_RESET(9);
static constexpr RegDetail DMA_CTRL_INT_EN(10);
static constexpr RegDetail DMA_CTRL_RUN_HW_EN(11);
static constexpr RegDetail DMA_CTRL_OB_BIG_ENDIAN(20);
static constexpr RegDetail DMA_CTRL_IB_BIG_ENDIAN(21);
static constexpr RegDetail DMA_CTRL_VALID_FIFO_EN(23);
static constexpr RegDetail DMA_CTRL_WR_INFO_EN(24);
static constexpr RegDetail DMA_CTRL_WR_INFO_HDR(25);
static constexpr RegDetail DMA_CTRL_WR_INFO_HDR_SZ(26, 31);

// Descriptor container register
static constexpr uint32_t DMA_DESC_CONT_REG = 0x4;
static constexpr RegDetail DMA_CUR_DESC_CONT(0, 7);
static constexpr RegDetail DMA_LAST_DESC_CONT(16, 23);

// Container address register (32 bit)
static constexpr uint32_t DMA_CONT_ADDR_REG = 0x8;

// Container size register
static constexpr uint32_t DMA_CONT_SIZE_REG = 0xc;
static constexpr RegDetail DMA_DESC_CONT_SIZE(0, 15);
static constexpr RegDetail DMA_MAX_DESC_CONTS(16, 23);

// Descriptor status register
static constexpr uint32_t DMA_DESC_STATUS_REG = 0x10;
static constexpr RegDetail DMA_DESC_VERSION(0, 3);
static constexpr RegDetail DMA_JOB_ID(16, 31);

// Input payload address register (32 bit)
static constexpr uint32_t DMA_INPUT_PAYLOAD_REG = 0x14;

// Output payload address register (32 bit)
static constexpr uint32_t DMA_OUTPUT_PAYLOAD_REG = 0x18;

// Output word count register (32 bit)
static constexpr uint32_t DMA_OUTPUT_WORD_COUNT_REG = 0x1c;

// Input word count register
static constexpr uint32_t DMA_INPUT_WORD_COUNT_REG = 0x20;
static constexpr RegDetail DMA_INPUT_WORD_COUNT(0, 14);

// Inbound buffer monitor control register
static constexpr uint32_t DMA_IB_BUF_MON_CTRL_REG = 0x24;
static constexpr RegDetail DMA_STATUS_CLEAR(0);
static constexpr RegDetail DMA_BUFFER_CNTR_CLEAR(1);
static constexpr RegDetail DMA_JOB_ID_FIFO_CLEAR(2);
static constexpr RegDetail DMA_BUF_END_SELECT(8, 9);
static constexpr RegDetail DMA_BUF_CNTR_EN(12);
static constexpr RegDetail DMA_BUF_TIMER_EN(13);
static constexpr RegDetail DMA_JOB_ID_FIFO_EN(14);
static constexpr RegDetail DMA_BUFFER_END_MASK_OB_END(16);
static constexpr RegDetail DMA_BUFFER_END_MASK_IB_END(17);
static constexpr RegDetail DMA_BUFFER_END_MASK_EXT_DMA_END(18);
static constexpr RegDetail DMA_BUFFER_END_MASK_DESC_BURST_END(19);

// Buffer monitor status register
static constexpr uint32_t DMA_BUF_MON_STATUS_REG = 0x28;
static constexpr RegDetail DMA_BUFFER_END_STATUS(0, 3);
static constexpr RegDetail DMA_BUFFER_END_STATUS_OB(0);
static constexpr RegDetail DMA_BUFFER_END_STATUS_IB(1);
static constexpr RegDetail DMA_BUFFER_END_STATUS_EXT_BUF_END(2);
static constexpr RegDetail DMA_BUFFER_END_STATUS_DESC_BURST_DONE(3);
static constexpr RegDetail DMA_BUFFER_END_INTS(16, 19);
static constexpr RegDetail DMA_BUFFER_END_INTS_OB(16);
static constexpr RegDetail DMA_BUFFER_END_INTS_IB(17);
static constexpr RegDetail DMA_BUFFER_END_INTS_EXT_BUF_END(18);
static constexpr RegDetail DMA_BUFFER_END_INTS_DESC_BURST_DONE(19);

// Buffer counter status register (32 bit)
static constexpr uint32_t DMA_BUFFER_COUNTER_STATUS_REG = 0x2c;

// Buffer timer status register (32 bit)
static constexpr uint32_t DMA_BUFFER_TIMER_STATUS_REG = 0x30;

// Descriptor Start Delays Register
static constexpr uint32_t DMA_DESC_START_DELAYS_REG = 0x38;
static constexpr RegDetail DMA_DESC_START_DELAY(0, 9);

// Extra Descriptors Control
static constexpr uint32_t DMA_EXTRA_DESC_CTRL_REG = 0x3c;
static constexpr RegDetail DMA_LAST_EXTRA_DESCRIPTOR(0, 7);
static constexpr RegDetail DMA_EXTRA_DESC_ENABLE(12);

// Debug registers
static constexpr uint32_t DMA_DEBUG_CTRL_REG = 0x40;
static constexpr uint32_t DMA_DEBUG_BUS0_STATUS_REG = 0x60;
static constexpr uint32_t DMA_DEBUG_BUS1_STATUS_REG = 0x64;
static constexpr uint32_t DMA_DEBUG_BUS2_STATUS_REG = 0x68;

// Job ID FIFO register
static constexpr uint32_t DMA_JOB_ID_FIFO_REG = 0x50;
static constexpr RegDetail DMA_JOB_ID_FIFO_CNT(0, 7);
static constexpr RegDetail DMA_JOB_ID_VALID(8);
static constexpr RegDetail DMA_JOB_ID_FIFO_OUT(16, 31);

// Replay Buffer Control register
static constexpr uint32_t DMA_REPLAY_BUF_CTRL_REG = 0x70;
static constexpr RegDetail DMA_REPLAY_MAX_DESC_BURST_MODE(0);
static constexpr RegDetail DMA_REPLAY_HW_OB_ADDR_GEN_MODE(4);
static constexpr RegDetail DMA_REPLAY_HW_OB_ADDR_DYN_MODE(5);
static constexpr RegDetail DMA_REPLAY_HW_OB_DESC_WORD5_EN(6);
static constexpr RegDetail DMA_REPLAY_START_HALT_EN(8);
static constexpr RegDetail DMA_REPLAY_INITIAL_START_HALT(9);
static constexpr RegDetail DMA_REPLAY_BUFFER_MODE(16);
static constexpr RegDetail DMA_REPLAY_TIMER_MODE(24);

// Replay Burst Value register
static constexpr uint32_t DMA_REPLAY_BURST_VAL_REG = 0x74;
static constexpr RegDetail DMA_REPLAY_MAX_DESC_BURST_VALUE(0, 7);
static constexpr RegDetail DMA_REPLAY_LOOPS(16, 23);
static constexpr RegDetail DMA_REPLAY_LOOPS_LAYER_PR(24, 31);

// Replay Descriptor Buffer Address register (32 bit)
static constexpr uint32_t DMA_REPLAY_DESC_MAIN_BUF_ADDR_REG = 0x78;
static constexpr RegDetail DMA_REPLAY_DESC_MAIN_BUF_ADDR(0, 31);

// Replay Descriptor Scratch Buffer Address register (32 bit)
static constexpr uint32_t DMA_REPLAY_DESC_SCRATCH_BUF_ADDR_REG = 0x7c;
static constexpr RegDetail DMA_REPLAY_DESC_SCRATCH_BUF_ADDR(0, 31);

// Replay OB Event Buffer Address register (32 bit)
static constexpr uint32_t DMA_REPLAY_OB_EVENT_BUF_ADDR_REG = 0x80;
static constexpr RegDetail DMA_REPLAY_OB_EVENT_BUF_ADDR(0, 31);

// Replay OB Event Scratch Address register (32 bit)
static constexpr uint32_t DMA_REPLAY_OB_EVENT_SCRATCH_ADDR_REG = 0x84;
static constexpr RegDetail DMA_REPLAY_OB_EVENT_SCRATCH_ADDR(0, 31);

// Replay OB Buffer Offset Address register
static constexpr uint32_t DMA_REPLAY_OB_BUF_ADDR_REG = 0x88;
static constexpr RegDetail DMA_REPLAY_OB_DESC_BUF_OFFSET(0, 2);
static constexpr RegDetail DMA_REPLAY_OB_EVENTS_BUF_OFFSET(16, 31);

// Replay Maximum OB Offset Buffers register
static constexpr uint32_t DMA_REPLAY_MAX_OB_BUFFERS_REG = 0x8c;
static constexpr RegDetail DMA_REPLAY_MAX_OB_DESC_BUFFERS(0, 11);
static constexpr RegDetail DMA_REPLAY_MAX_OB_EVENTS_BUFFERS(16, 31);

// Replay Descriptor Word5 register
static constexpr uint32_t DMA_REPLAY_DESC_WORD5_REG = 0x90;
static constexpr RegDetail DMA_REPLAY_DESC_WORD5(0, 31);

// DMA Interrupt Interval register
static constexpr uint32_t DMA_INTERRUPT_INTERVAL_REG = 0x94;
static constexpr RegDetail DMA_INBOUND_INTERVAL(0, 7);
static constexpr RegDetail DMA_OUTBOUND_INTERVAL(16, 23);

// DMA OB PLD Clear Size Register
static constexpr uint32_t DMA_OB_PLD_CLEAR_SIZE_REG = 0x98;
static constexpr RegDetail DMA_OB_PLD_CLR_SIZE(0, 27);
static constexpr RegDetail DMA_OB_PLD_CLR_EN(31);

// DMA Reset Control register
static constexpr uint32_t DMA_RESET_CTRL_REG = 0xa0;
static constexpr RegDetail DMA_LOGIC_RESET(0);
static constexpr RegDetail DMA_IB_RESET(1);
static constexpr RegDetail DMA_OB_RESET(2);
static constexpr RegDetail DMA_FORCE_BURST_RESUME(4);

}  // namespace akida
