//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
#include "pcie-endpoint.h"
#include <src/renode_bus.h>

// ── Free-running clock ────────────────────────────────────────────────────────

void PcieEndpoint::startClock()
{
    clock_running = true;
    clock_thread = std::thread(&PcieEndpoint::clockLoop, this);
}

void PcieEndpoint::stopClock()
{
    clock_running = false;
    if(clock_thread.joinable()) {
        clock_thread.join();
    }
}

// clockLoop — runs in a dedicated OS thread, ticks the RTL continuously.
//
// Socket ownership:
//   main_fd   — owned by the main thread (RenodeAgent::simulate). This thread
//               never reads from main_fd; it only reads/writes sender_fd.
//   sender_fd — owned by this thread (DMA requests and interrupt notifications).
//
// Transaction protocol:
//   Main thread posts PendingTxn under txn_mutex and blocks on resp_cv.
//   This thread picks up the transaction at the start of the next clock cycle,
//   drives the BAR signals, ticks until bar_rdy, then posts TxnResponse and
//   notifies resp_cv.  The RTL never sees the clock stop.
void PcieEndpoint::clockLoop()
{
    while(clock_running) {
        // 1. Inject pending transaction at start of cycle (under txn_mutex).
        {
            std::lock_guard<std::mutex> lk(txn_mutex);
            if(pending_txn.valid) {
                *bar_addr = pending_txn.addr;
                *bar_wen  = pending_txn.is_read ? 0 : 1;
                *bar_ren  = pending_txn.is_read ? 1 : 0;
                if(!pending_txn.is_read) {
                    *bar_wdata = (uint32_t)pending_txn.wdata;
                }
                evaluateModel();
            }
        }

        // 2. Rising edge.
        *clk = 1;
        evaluateModel();

        // 3. Falling edge.
        *clk = 0;
        evaluateModel();

        // 4. Check transaction completion.
        {
            std::lock_guard<std::mutex> lk(txn_mutex);
            if(pending_txn.valid && *bar_rdy) {
                uint64_t rdata = *bar_rdata;

                // Deassert BAR control signals before the next cycle.
                *bar_wen = 0;
                *bar_ren = 0;
                evaluateModel();

                pending_txn.valid = false;

                {
                    std::lock_guard<std::mutex> rlk(resp_mutex);
                    txn_response = {rdata, true};
                }
                resp_cv.notify_one();
            }
        }

        // 5. DMA (device → host).  Checked every cycle; sender_fd is
        //    exclusively owned by this thread so no locking needed here.
        handleDma();

        // 6. Interrupts — edge-detect irq and send interrupt action over
        //    sender_fd via the existing handleInterrupts() mechanism.
        agent->handleInterrupts();
    }
}

// ── MWr — posted write ────────────────────────────────────────────────────────
//
// Called by RenodeAgent::writeToBus() on the main thread.
// Posts the transaction and blocks until the clock thread completes it.
// Does NOT call tick() — the clock thread does all ticking.

void PcieEndpoint::write(int width, uint64_t addr, uint64_t value)
{
    {
        std::lock_guard<std::mutex> lk(txn_mutex);
        pending_txn = {width, addr, value, /*is_read=*/false, /*valid=*/true};
    }

    std::unique_lock<std::mutex> lk(resp_mutex);
    resp_cv.wait(lk, [this]{ return txn_response.valid; });
    txn_response.valid = false;
}

// ── MRd → CplD — non-posted read, completion with data ───────────────────────

uint64_t PcieEndpoint::read(int width, uint64_t addr)
{
    {
        std::lock_guard<std::mutex> lk(txn_mutex);
        pending_txn = {width, addr, 0, /*is_read=*/true, /*valid=*/true};
    }

    std::unique_lock<std::mutex> lk(resp_mutex);
    resp_cv.wait(lk, [this]{ return txn_response.valid; });
    uint64_t result = txn_response.rdata;
    txn_response.valid = false;
    return result;
}

// ── DMA initiator (device → host) ─────────────────────────────────────────────
//
// Called from clockLoop() only — clock thread exclusively owns sender_fd.
// Detects dma_wen / dma_ren asserted by the RTL and forwards the transaction
// to the QEMU proxy (pci_dma_write / pci_dma_read) via the co-sim protocol.

void PcieEndpoint::handleDma()
{
    if(*dma_wen) {
        // pushDoubleWordToAgent sends pushDoubleWord over sender_fd and
        // blocks until pushConfirmation arrives on sender_fd.
        agent->pushDoubleWordToAgent(*dma_addr, *dma_wdata);

        // Pulse dma_rdy for one cycle to acknowledge to the RTL.
        *dma_rdy = 1;
        *clk = 1; evaluateModel();
        *clk = 0; evaluateModel();
        *dma_rdy = 0;
        evaluateModel();
    }
    else if(*dma_ren) {
        // requestDoubleWordFromAgent sends getDoubleWord over sender_fd and
        // blocks until the proxy responds with the data from pci_dma_read().
        uint32_t data = (uint32_t)agent->requestDoubleWordFromAgent(*dma_addr);
        *dma_rdata = data;

        *dma_rdy = 1;
        *clk = 1; evaluateModel();
        *clk = 0; evaluateModel();
        *dma_rdy = 0;
        evaluateModel();
    }
}

// ── Reset ─────────────────────────────────────────────────────────────────────

void PcieEndpoint::reset()
{
    stopClock();

#ifdef RESET_ACTIVE_LOW
    *rst = 0; evaluateModel();
    *rst = 1; evaluateModel();
#else
    *rst = 1; evaluateModel();
    *rst = 0; evaluateModel();
#endif

    startClock();
}

// ── tick / timeoutTick — used by RenodeAgent for TickClock handling ───────────
//
// In the free-running model the clock thread drives all ticks. These methods
// are still required by the BaseTargetBus interface for TickClock messages sent
// by the Renode-side protocol (not used when the host is QEMU), and for the
// tickCounter used by handleRequest(). They are no-ops for the QEMU path.

void PcieEndpoint::tick(bool countEnable, uint64_t steps)
{
    if(countEnable) {
        tickCounter += steps;
    }
}

void PcieEndpoint::timeoutTick(uint8_t* signal, uint8_t expectedValue, int timeout)
{
    // Not called in the free-running model; clock thread manages all waits.
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
