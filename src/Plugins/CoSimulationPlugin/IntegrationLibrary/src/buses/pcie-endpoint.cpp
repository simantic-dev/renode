//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
#include "pcie-endpoint.h"
#include <src/renode_bus.h>

// ── Clocking ──────────────────────────────────────────────────────────────────
//
// tick() is the single point that advances the RTL clock. It is called both by
// the TickClock(N) handler (free-running advance between transactions) and by
// write()/read() (ticking until the BAR handshake completes). DMA and interrupt
// servicing happen here so they are observed across the free-running cycles too.

void PcieEndpoint::tick(bool countEnable, uint64_t steps)
{
    for(uint64_t i = 0; i < steps; i++) {
        *clk = 1;
        evaluateModel();
        *clk = 0;
        evaluateModel();

        // RTL-initiated DMA and interrupts are serviced every cycle, so a
        // free-running TickClock(N) advance still drains the device's engines.
        handleDma();
        agent->handleInterrupts();
    }

    if(countEnable) {
        tickCounter += steps;
    }
}

void PcieEndpoint::timeoutTick(uint8_t* signal, uint8_t expectedValue, int timeout)
{
    do {
        tick(true);
        timeout--;
    } while((*signal != expectedValue) && timeout > 0);

    if(timeout == 0) {
        throw "PCIe endpoint: timeout waiting for bar_rdy";
    }
}

// ── MWr — posted write ────────────────────────────────────────────────────────

void PcieEndpoint::write(int width, uint64_t addr, uint64_t value)
{
    *bar_addr  = addr;
    *bar_wdata = (uint32_t)value;
    *bar_wen   = 1;
    *bar_ren   = 0;
    tick(true);

    *bar_wen = 0;
    if(!*bar_rdy) {
        timeoutTick(bar_rdy, 1, DEFAULT_TIMEOUT);
    }
    tick(true);
}

// ── MRd → CplD — non-posted read, completion with data ───────────────────────

uint64_t PcieEndpoint::read(int width, uint64_t addr)
{
    *bar_addr = addr;
    *bar_ren  = 1;
    *bar_wen  = 0;
    tick(true);

    if(!*bar_rdy) {
        timeoutTick(bar_rdy, 1, DEFAULT_TIMEOUT);
    }
    uint64_t result = *bar_rdata;

    *bar_ren = 0;
    tick(true);
    return result;
}

// ── DMA initiator (device → host) ─────────────────────────────────────────────
//
// Called inside tick() once per cycle. When the RTL asserts dma_wen / dma_ren,
// the transaction is forwarded to the QEMU proxy (pci_dma_write / pci_dma_read)
// via the co-sim protocol. pushDoubleWordToAgent / requestDoubleWordFromAgent
// run synchronously here (nested receive on the main socket) — single-threaded,
// so no race with the simulate() loop. dma_rdy is held for the cycle in which
// the transfer completes; the RTL deasserts its request on seeing it.

void PcieEndpoint::handleDma()
{
    if(*dma_wen) {
        agent->pushDoubleWordToAgent(*dma_addr, *dma_wdata);
        *dma_rdy = 1;
        evaluateModel();
    }
    else if(*dma_ren) {
        uint32_t data = (uint32_t)agent->requestDoubleWordFromAgent(*dma_addr);
        *dma_rdata = data;
        *dma_rdy = 1;
        evaluateModel();
    }
    else if(*dma_rdy) {
        // Clear the completion flag the cycle after the handshake.
        *dma_rdy = 0;
        evaluateModel();
    }
}

// ── Reset ─────────────────────────────────────────────────────────────────────

void PcieEndpoint::reset()
{
#ifdef RESET_ACTIVE_LOW
    *rst = 0;
    tick(true);
    *rst = 1;
    tick(true);
#else
    *rst = 1;
    tick(true);
    *rst = 0;
    tick(true);
#endif
}

// ── Signal connectivity check ─────────────────────────────────────────────────

bool PcieEndpoint::areSignalsConnected()
{
    return isSignalConnected(clk,       "clk")
        && isSignalConnected(rst,       "rst")
        && isSignalConnected(bar_addr,  "bar_addr")
        && isSignalConnected(bar_wdata, "bar_wdata")
        && isSignalConnected(bar_rdata, "bar_rdata")
        && isSignalConnected(bar_wen,   "bar_wen")
        && isSignalConnected(bar_ren,   "bar_ren")
        && isSignalConnected(bar_rdy,   "bar_rdy")
        && isSignalConnected(dma_addr,  "dma_addr")
        && isSignalConnected(dma_wdata, "dma_wdata")
        && isSignalConnected(dma_rdata, "dma_rdata")
        && isSignalConnected(dma_wen,   "dma_wen")
        && isSignalConnected(dma_ren,   "dma_ren")
        && isSignalConnected(dma_rdy,   "dma_rdy")
        && isSignalConnected(irq,       "irq");
}
