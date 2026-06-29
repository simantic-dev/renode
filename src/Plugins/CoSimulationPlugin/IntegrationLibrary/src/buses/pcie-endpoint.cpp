//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
#include "pcie-endpoint.h"
#include <src/renode_bus.h>

// ── Clocking ──────────────────────────────────────────────────────────────────

void PcieEndpoint::tick(bool countEnable, uint64_t steps)
{
    for(uint64_t i = 0; i < steps; i++) {
        setSignal<uint8_t>(clk, 1);
        setSignal<uint8_t>(clk, 0);
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
        throw "PCIe endpoint: operation timeout waiting for bar_rdy";
    }
}

// ── MWr — posted write (no completion required) ───────────────────────────────

void PcieEndpoint::write(int width, uint64_t addr, uint64_t value)
{
    setSignal<uint64_t>(bar_addr,  addr);
    setSignal<uint32_t>(bar_wdata, (uint32_t)value);
    setSignal<uint8_t> (bar_wen,   1);
    setSignal<uint8_t> (bar_ren,   0);
    tick(true);

    setSignal<uint8_t>(bar_wen, 0);

    // Wait for RTL to acknowledge.  bar_rdy may already be high on zero-wait
    // combinational registers; otherwise spin until asserted.
    if(!*bar_rdy) {
        timeoutTick(bar_rdy, 1, DEFAULT_TIMEOUT);
    }
    tick(true);
}

// ── MRd → CplD — non-posted read, completion with data ───────────────────────

uint64_t PcieEndpoint::read(int width, uint64_t addr)
{
    setSignal<uint64_t>(bar_addr, addr);
    setSignal<uint8_t> (bar_ren,  1);
    setSignal<uint8_t> (bar_wen,  0);
    tick(true);

    if(!*bar_rdy) {
        timeoutTick(bar_rdy, 1, DEFAULT_TIMEOUT);
    }

    uint64_t result = *bar_rdata;

    setSignal<uint8_t>(bar_ren, 0);
    tick(true);

    return result;
}

// ── Reset ─────────────────────────────────────────────────────────────────────

void PcieEndpoint::reset()
{
#ifdef RESET_ACTIVE_LOW
    setSignal<uint8_t>(rst, 0);
    tick(true);
    setSignal<uint8_t>(rst, 1);
    tick(true);
#else
    setSignal<uint8_t>(rst, 1);
    tick(true);
    setSignal<uint8_t>(rst, 0);
    tick(true);
#endif
}

// ── DMA initiator (device → host) ─────────────────────────────────────────────
//
// Called from the user's sim_main loop after each tick().  Checks if the RTL
// has initiated a DMA transaction and, if so, forwards it to the host via the
// co-sim protocol (pushDoubleWord → pci_dma_write; getDoubleWord → pci_dma_read).
//
// Only 32-bit DMA is supported for the MVP; extend to 64-bit with
// pushQuadWord / getQuadWord when needed.

void PcieEndpoint::handleDma()
{
    if(*dma_wen) {
        // MWr from device to host — posted, no completion TLP.
        // pushToAgent blocks until it receives pushConfirmation from the proxy.
        agent->pushDoubleWordToAgent(*dma_addr, *dma_wdata);

        // Pulse dma_rdy to tell the RTL the write landed.
        setSignal<uint8_t>(dma_rdy, 1);
        tick(false);
        setSignal<uint8_t>(dma_rdy, 0);
    }
    else if(*dma_ren) {
        // MRd from device — non-posted.  requestFromAgent blocks until the
        // proxy responds with the data read via pci_dma_read().
        uint32_t data = (uint32_t)agent->requestDoubleWordFromAgent(*dma_addr);
        setSignal<uint32_t>(dma_rdata, data);

        // Pulse dma_rdy to tell the RTL the data is valid.
        setSignal<uint8_t>(dma_rdy, 1);
        tick(false);
        setSignal<uint8_t>(dma_rdy, 0);
    }
}

// ── Signal connectivity check ─────────────────────────────────────────────────

bool PcieEndpoint::areSignalsConnected()
{
    return isSignalConnected(clk,      "clk")
        && isSignalConnected(rst,      "rst")
        && isSignalConnected(bar_addr, "bar_addr")
        && isSignalConnected(bar_wdata,"bar_wdata")
        && isSignalConnected(bar_rdata,"bar_rdata")
        && isSignalConnected(bar_wen,  "bar_wen")
        && isSignalConnected(bar_ren,  "bar_ren")
        && isSignalConnected(bar_rdy,  "bar_rdy")
        && isSignalConnected(dma_addr, "dma_addr")
        && isSignalConnected(dma_wdata,"dma_wdata")
        && isSignalConnected(dma_rdata,"dma_rdata")
        && isSignalConnected(dma_wen,  "dma_wen")
        && isSignalConnected(dma_ren,  "dma_ren")
        && isSignalConnected(dma_rdy,  "dma_rdy")
        && isSignalConnected(irq,      "irq");
}
