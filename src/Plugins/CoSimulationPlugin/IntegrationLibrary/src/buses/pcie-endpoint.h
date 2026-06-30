//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
#ifndef PcieEndpoint_H
#define PcieEndpoint_H
#include "bus.h"

// PCIe endpoint bus class — transaction layer for a Verilated PCIe endpoint.
//
// Single-threaded, mirroring the proven FastVDMA / FPGA-ISP co-sim pattern.
// The RTL clock is free-running from the *host's* perspective because the
// QEMU proxy emits TickClock(N) on a virtual-time timer (the same role
// Renode's LimitTimer plays). Between BAR accesses the RTL keeps clocking;
// DMA and interrupts are serviced inside tick() across those N cycles.
//
// TLP mapping:
//   tickClock(N)  → tick(N)  — free-running clock advance; DMA + IRQ checked each cycle
//   MWr  (host→device)       → write()  drives BAR signals, ticks until bar_rdy
//   MRd  (host→device)       → read()   drives BAR signals, returns bar_rdata at bar_rdy
//   DMA write (device→host)  → handleDma() detects dma_wen → pushDoubleWordToAgent
//   DMA read  (device←host)  → handleDma() detects dma_ren → requestDoubleWordFromAgent
//   MSI-X                    → agent->handleInterrupts() each cycle; register irq in sim_main.cpp
//
// All pushToAgent / requestFromAgent calls happen synchronously inside the
// tick() call stack, so the nested receive() (which always reads the main
// socket) is naturally serialized with the simulate() loop — no threads, no
// cross-socket races.
//
// Usage in sim_main.cpp:
//   RenodeAgent* Init() {
//       auto* agent = new RenodeAgent();
//       auto* bus   = new PcieEndpoint();
//       bus->clk = &top->clk;  // wire all signal pointers
//       // ...
//       agent->addBus(bus);                      // calls setAgent — wires evaluateModel
//       agent->registerInterrupt(bus->irq, 0);   // MSI-X vector 0
//       return agent;
//   }
struct PcieEndpoint : public BaseTargetBus
{
    virtual void     tick(bool countEnable, uint64_t steps);
    virtual void     write(int width, uint64_t addr, uint64_t value);
    virtual uint64_t read(int width, uint64_t addr);
    virtual void     timeoutTick(uint8_t* signal, uint8_t expectedValue, int timeout);
    virtual void     reset();
    virtual bool     areSignalsConnected();

    // ── BAR slave interface (host → device) ──────────────────────────────
    uint8_t  *clk      = nullptr;
    uint8_t  *rst      = nullptr;
    uint64_t *bar_addr = nullptr;   // byte offset within BAR
    uint32_t *bar_wdata = nullptr;  // write data  (32-bit — PCIe MWr payload)
    uint32_t *bar_rdata = nullptr;  // read data   (32-bit — CplD payload)
    uint8_t  *bar_wen  = nullptr;   // asserted for the setup cycle on MWr
    uint8_t  *bar_ren  = nullptr;   // asserted on MRd; held until bar_rdy
    uint8_t  *bar_rdy  = nullptr;   // RTL asserts when the cycle is complete

    // ── DMA initiator interface (device → host) ───────────────────────────
    // RTL holds dma_wen or dma_ren with valid dma_addr / dma_wdata until the
    // bus class drives dma_rdy for one cycle on completion.
    uint64_t *dma_addr  = nullptr;
    uint32_t *dma_wdata = nullptr;
    uint32_t *dma_rdata = nullptr;
    uint8_t  *dma_wen   = nullptr;
    uint8_t  *dma_ren   = nullptr;
    uint8_t  *dma_rdy   = nullptr;

    // ── MSI-X ─────────────────────────────────────────────────────────────
    // Wire to the RTL's interrupt output; register with
    // agent->registerInterrupt(bus->irq, 0) in sim_main.cpp.
    uint8_t  *irq = nullptr;

private:
    void handleDma();   // serviced inside tick() each cycle
};
#endif
