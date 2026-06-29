//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
#ifndef PcieEndpoint_H
#define PcieEndpoint_H
#include "bus.h"

// PCIe endpoint bus class — models the TLP transaction layer for a PCIe
// endpoint device at the abstraction level used by Renode co-simulation:
//
//   MWr  (posted write)      → write(width, addr, value)
//   MRd  (non-posted read)   → read(width, addr) → value
//   DMA write (device→host)  → handleDma() detects dma_wen, calls pushDoubleWordToAgent
//   DMA read  (device←host)  → handleDma() detects dma_ren, calls requestDoubleWordFromAgent
//   MSI-X                    → registerInterrupt(irq, 0) + handleInterrupts() in main loop
//
// Wire your Verilated model's port pointers to the signal members before
// calling setAgent(), then call handleDma() and agent->handleInterrupts()
// in your main simulation loop after each tick.
struct PcieEndpoint : public BaseTargetBus
{
    virtual void tick(bool countEnable, uint64_t steps);
    virtual void write(int width, uint64_t addr, uint64_t value);
    virtual uint64_t read(int width, uint64_t addr);
    virtual void reset();
    virtual bool areSignalsConnected();

    // Called from the user's sim_main loop after each tick.
    // Detects RTL-initiated DMA and forwards to the host via pushToAgent /
    // requestFromAgent (which forward to pci_dma_write / pci_dma_read in the
    // QEMU proxy).
    void handleDma();

    void timeoutTick(uint8_t* signal, uint8_t expectedValue, int timeout);

    // ── BAR slave interface (host → device, target) ──────────────────────
    uint8_t  *clk        = nullptr;
    uint8_t  *rst        = nullptr;
    uint64_t *bar_addr   = nullptr;   // byte address within the BAR
    uint32_t *bar_wdata  = nullptr;   // write data (32-bit for PCIe MWr)
    uint32_t *bar_rdata  = nullptr;   // read data  (32-bit for CplD)
    uint8_t  *bar_wen    = nullptr;   // asserted for one cycle on MWr
    uint8_t  *bar_ren    = nullptr;   // asserted on MRd; hold until bar_rdy
    uint8_t  *bar_rdy    = nullptr;   // RTL asserts when response is ready

    // ── DMA initiator interface (device → host) ───────────────────────────
    // RTL drives dma_wen or dma_ren for one cycle with a valid dma_addr/
    // dma_wdata; the bus class asserts dma_rdy for one cycle on completion.
    uint64_t *dma_addr   = nullptr;   // host physical address
    uint32_t *dma_wdata  = nullptr;   // data for DMA write
    uint32_t *dma_rdata  = nullptr;   // data returned by DMA read
    uint8_t  *dma_wen    = nullptr;   // initiate a DMA write to host
    uint8_t  *dma_ren    = nullptr;   // initiate a DMA read from host
    uint8_t  *dma_rdy    = nullptr;   // pulsed by bus class when DMA completes

    // ── MSI-X ─────────────────────────────────────────────────────────────
    // Call agent->registerInterrupt(irq, 0) in sim_main.cpp after setAgent().
    // handleInterrupts() in the main loop detects edges and sends the
    // interrupt action to the QEMU proxy which calls msix_notify().
    uint8_t  *irq        = nullptr;
};
#endif
