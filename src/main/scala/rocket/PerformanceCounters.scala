// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.tile._
import freechips.rocketchip.config.Parameters
import chisel3.{withClock, VecInit}
import scala.collection.mutable.ArrayBuffer

trait HasAccumEventIO {
  //only doing this to match how the core IO is built
  val accumEvent = Bool().asInput
}

trait HasPTWPerfEvents {
  val perf = new Bundle {
    val l2miss = Bool()
    val l2hit = Bool()
    val pte_miss = Bool()
    val pte_hit = Bool()
  }.asOutput
}

trait HasICachePerfEvents {
  val perf = new Bundle {
    val acquire = Bool()
  }.asOutput
}

trait HasHellaCachePerfEvents {
  val perf = new Bundle {
    val canAcceptStoreThenLoad = Bool()
    val canAcceptStoreThenRMW = Bool()
    val canAcceptLoadThenLoad = Bool()
    val storeBufferEmptyAfterLoad = Bool()
    val storeBufferEmptyAfterStore = Bool()
  }.asInput
}

trait HasFrontEndPerfEvents {
  val perf = new Bundle {
      val acquire = Bool()
      val tlbMiss = Bool()
    }.asInput
}

class PerfCounterIO(implicit p: Parameters) extends CoreBundle
    with HasCoreParameters {
  val eventSel = UInt(OUTPUT, xLen)
  val inc = UInt(INPUT, log2Ceil(1+retireWidth))
}

abstract class PerformanceCounters(perfEventSets: EventSets = new EventSets(),
  csrFile: CSRFile, nPerfCounters: Int) extends CSRHardware{

  val firstCtr = CSRs.cycle
  val firstCtrH = CSRs.cycleh
  val firstHPC = CSRs.hpmcounter3
  val firstHPCH = CSRs.hpmcounter3h
  val firstHPE = CSRs.mhpmevent3
  val firstMHPC = CSRs.mhpmcounter3
  val firstMHPCH = CSRs.mhpmcounter3h
  val firstHPM = 3
  val nHPM = CSR.nCtr - firstHPM
  val hpmWidth = 40

  val delegable_counters = ((BigInt(1) << (nPerfCounters + firstHPM)) - 1).U
  val (reg_mcounteren, read_mcounteren) = {
    val reg = Reg(UInt(32.W))
    (reg, Mux(csrFile.usingUser, reg & delegable_counters, 0.U))
  }
  val (reg_scounteren, read_scounteren) = {
    val reg = Reg(UInt(32.W))
    (reg, Mux(csrFile.usingSupervisor, reg & delegable_counters, 0.U))
  }

  val reg_mcountinhibit = RegInit(0.U((firstHPM + nPerfCounters).W))
  csrFile.io.inhibit_cycle := reg_mcountinhibit(0)

  //counters that never change
  val reg_instret = WideCounter(64, csrFile.io.retire, inhibit = reg_mcountinhibit(2))
  val reg_cycle = if (csrFile.enableCommitLog) WideCounter(64, csrFile.io.retire, inhibit = reg_mcountinhibit(0))
    else withClock(csrFile.io.ungated_clock) { WideCounter(64, !csrFile.io.csr_stall, inhibit = reg_mcountinhibit(0)) }
  csrFile.io.time := reg_cycle

  //user assignable counters
  val reg_hpmevent = csrFile.io.counters.map(c => Reg(init = UInt(0, csrFile.xLen)))
  //Tell the IO what event needs to be pulled into the CSR
  (csrFile.io.counters zip reg_hpmevent) foreach { case (c, e) => c.eventSel := e }
  //Create all the counting registers and connect them to the inc signals
  val reg_hpmcounter = csrFile.io.counters.zipWithIndex.map { case (c, i) =>
    WideCounter(hpmWidth, c.inc, reset = false, inhibit = reg_mcountinhibit(firstHPM+i)) }

  def buildMappings() = {
    csrFile.read_mapping += CSRs.mcountinhibit -> reg_mcountinhibit
    csrFile.read_mapping += CSRs.mcycle -> reg_cycle
    csrFile.read_mapping += CSRs.minstret -> reg_instret

    if (csrFile.usingUser) {
      csrFile.read_mapping += CSRs.mcounteren -> read_mcounteren
      csrFile.read_mapping += CSRs.cycle -> reg_cycle
      csrFile.read_mapping += CSRs.instret -> reg_instret
    }

    if (csrFile.usingSupervisor) {
      csrFile.read_mapping += CSRs.scounteren -> read_scounteren
    }

    if (csrFile.xLen == 32) {
      csrFile.read_mapping += CSRs.mcycleh -> (reg_cycle >> 32)
      csrFile.read_mapping += CSRs.minstreth -> (reg_instret >> 32)
      if (csrFile.usingUser) {
        csrFile.read_mapping += CSRs.cycleh -> (reg_cycle >> 32)
        csrFile.read_mapping += CSRs.instreth -> (reg_instret >> 32)
      }
    }
  }

  def buildDecode() = {
    if (csrFile.usingSupervisor) {
      when (csrFile.decoded_addr(CSRs.scounteren)) { reg_scounteren := csrFile.wdata }
    }

    if (csrFile.usingUser) {
      when (csrFile.decoded_addr(CSRs.mcounteren)) { reg_mcounteren := csrFile.wdata }
    }

    when(csrFile.csr_wen) {
      when (csrFile.decoded_addr(CSRs.mcountinhibit)) { reg_mcountinhibit := csrFile.wdata & ~2.U(csrFile.xLen.W) }  // mcountinhibit bit [1] is tied zero
      writeCounter(CSRs.mcycle, reg_cycle, csrFile.wdata)
      writeCounter(CSRs.minstret, reg_instret, csrFile.wdata)
    }

    for (io_dec <- csrFile.io.decode) {
      val counter_addr = io_dec.csr(log2Ceil(read_mcounteren.getWidth)-1, 0)
      val allow_counter = (csrFile.reg_mstatus.prv > PRV.S || read_mcounteren(counter_addr)) &&
        (!csrFile.usingSupervisor || csrFile.reg_mstatus.prv >= PRV.S || read_scounteren(counter_addr))

      //this never changes, nCtr is a counter. Reguardless of what performance counters
      //are created, these addresses will always be considered valid
      when((io_dec.csr.inRange(firstCtr, firstCtr + CSR.nCtr) || io_dec.csr.inRange(firstCtrH, firstCtrH + CSR.nCtr))
        && !allow_counter) {
        io_dec.read_illegal := true.B
      }
    }

  }

  protected def writeCounter(lo: Int, ctr: WideCounter, wdata: UInt) = {
    if (csrFile.xLen == 32) {
      val hi = lo + CSRs.mcycleh - CSRs.mcycle
      when (csrFile.decoded_addr(lo)) { ctr := Cat(ctr(ctr.getWidth-1, 32), wdata) }
      when (csrFile.decoded_addr(hi)) { ctr := Cat(wdata(ctr.getWidth-33, 0), ctr(31, 0)) }
    } else {
      when (csrFile.decoded_addr(lo)) { ctr := wdata(ctr.getWidth-1, 0) }
    }
  }
}

class DirectPerformanceCounters(perfEventSets: EventSets = new EventSets(),
  csrFile: CSRFile, nPerfCounters: Int) extends PerformanceCounters(perfEventSets, csrFile, nPerfCounters) {

  //This build mappings directly connects the performance counters to the outside world
  //by memory mapping them
  override def buildMappings() = {
    //do the mapping for the non-user assignable counters
    super.buildMappings()

    //assign all the addresses for accessing the event and counter registers
    for (((e, c), i) <- (reg_hpmevent.padTo(nHPM, UInt(0))
                         zip reg_hpmcounter.map(x => x: UInt).padTo(nHPM, UInt(0))) zipWithIndex) {
      csrFile.read_mapping += (i + firstHPE) -> e // mhpmeventN
      csrFile.read_mapping += (i + firstMHPC) -> c // mhpmcounterN
      if (csrFile.usingUser) csrFile.read_mapping += (i + firstHPC) -> c // hpmcounterN
      if (csrFile.xLen == 32) {
        csrFile.read_mapping += (i + firstMHPCH) -> (c >> 32) // mhpmcounterNh
        if (csrFile.usingUser) csrFile.read_mapping += (i + firstHPCH) -> (c >> 32) // hpmcounterNh
      }
    }
  }

  override def buildDecode() = {
    //build the decode for the non-user assignable registers
    super.buildDecode()

    //tell the CSR what to do with signal information for the counters
    when(csrFile.csr_wen) {
      for (((e, c), i) <- (reg_hpmevent zip reg_hpmcounter) zipWithIndex) {
        writeCounter(i + firstMHPC, c, csrFile.wdata)
        when (csrFile.decoded_addr(i + firstHPE)) { e := perfEventSets.maskEventSelector(csrFile.wdata) }
      }
    }

    perfEventSets.print()
  }
}

class PCPerformanceCounters (perfEventSets: EventSets = new EventSets(),
  csrFile: CSRFile, nPerfCounters: Int) extends DirectPerformanceCounters(perfEventSets, csrFile, nPerfCounters) {
  val reg_addressInhibit = Reg(Bool())
  reg_addressInhibit := true.B //assume that we're going to inhibit things

  override val reg_hpmcounter = csrFile.io.counters.zipWithIndex.map { case (c, i) =>
    WideCounter(hpmWidth, c.inc, reset = false, inhibit = (reg_mcountinhibit(firstHPM+i) || reg_addressInhibit)) }

  val addr_PCStart = 0x80000000L
  //val reg_PCStart = RegInit(0.U(csrFile.vaddrBitsExtended.W))

  val addr_PCEnd = 0x90000000L
  //val reg_PCEnd = RegInit(0.U(csrFile.vaddrBitsExtended.W))

  val inAddressRange = csrFile.io.pc > addr_PCStart.U && csrFile.io.pc < addr_PCEnd.U
  //val registersEmpty = reg_PCStart === 0.U && reg_PCEnd === 0.U

  //when we're between the addresses or the registers are empty, count
  when(inAddressRange) {
    reg_addressInhibit := false.B
  }

  /* override def buildMappings() = {
    super.buildMappings()

    csrFile.read_mapping += addr_PCStart -> reg_PCStart
    csrFile.read_mapping += addr_PCEnd -> reg_PCEnd
  }

  override def buildDecode() = {
    super.buildDecode()

    when(csrFile.csr_wen) {
      when (csrFile.decoded_addr(addr_PCStart)) {
        reg_PCStart := csrFile.wdata(csrFile.vaddrBitsExtended-1,0)
      }

      when(csrFile.decoded_addr(addr_PCEnd)) {
        reg_PCEnd := csrFile.wdata(csrFile.vaddrBitsExtended-1,0)
      }
    }
  } */
}


class StatisticalPerformanceCounters(perfEventSets: EventSets = new EventSets(),
  csrFile: CSRFile, nPerfCounters: Int, nStatsCounters: Int) extends PerformanceCounters(perfEventSets, csrFile, nPerfCounters) {

  //build out registers for storing events
  val reg_eventStorage = RegInit(VecInit(Seq.fill(nStatsCounters)(0.U(csrFile.xLen.W))))

  //build out registers for storing counters
  val reg_counterStorage = Reg(Vec(nStatsCounters, UInt(hpmWidth.W)))

  //each real counter will be related to the i + (nPerfCounter)th counters
  require(nStatsCounters % nPerfCounters == 0)
  val statsLen = nStatsCounters/nPerfCounters

  val counterMatrix = for(i <- 0 until nPerfCounters) yield {
    val offset = i * statsLen
    for (j <- 0 until statsLen) yield {
      (reg_eventStorage(offset + j), reg_counterStorage(offset + j))
    }
  }

  //tell the event management it's time to do a swap
  val triggerAccum = (reg_cycle % 10000) === 0.U
  val triggerSwap = RegNext(triggerAccum)

  //circular counter to point to the current event being sampled
  val counter = Counter(statsLen)
  when(triggerSwap) {
    counter.inc
  }

  for(((e,c), i) <- (reg_hpmevent zip reg_hpmcounter).zipWithIndex)
    buildEventManagement(counterMatrix(i), e, c)

  private def buildEventManagement(storage: Seq[(UInt, UInt)], realEvent: UInt, realCounter: WideCounter) = {

    //mux for finding the event
    val currentEventMux = MuxLookup(counter.value, storage(0)._1,
      (for (((e,_), i) <- storage.zipWithIndex) yield {
        (i.U -> e)
      }).toSeq)

    //The real event and real counter are set to what the mux points to
    realEvent := currentEventMux

    //when the swap is triggered
    for(i <- 0 until storage.length) {
      when(counter.value === i && triggerAccum) {
        storage(i)._2 := storage(i)._2 + (realCounter * storage.length)
      }
    }

    //once cycle later, set the registers to 0 and incriment the counter
    when(triggerSwap) {
      realEvent := 0.U
      realCounter := 0.U
    }
  }

  override def buildMappings() = {
    //build the mappings for the non-user assignable counters
    super.buildMappings()

    //assign all the addresses for accessing the event and counter registers
    for (((e, c), i) <- (reg_eventStorage.padTo(nHPM, UInt(0))
                         zip reg_counterStorage.padTo(nHPM, UInt(0))) zipWithIndex) {
      csrFile.read_mapping += (i + firstHPE) -> e // mhpmeventN
      csrFile.read_mapping += (i + firstMHPC) -> c // mhpmcounterN
      if (csrFile.usingUser) csrFile.read_mapping += (i + firstHPC) -> c // hpmcounterN
      if (csrFile.xLen == 32) {
        csrFile.read_mapping += (i + firstMHPCH) -> (c >> 32) // mhpmcounterNh
        if (csrFile.usingUser) csrFile.read_mapping += (i + firstHPCH) -> (c >> 32) // hpmcounterNh
      }
    }
  }

  override def buildDecode() = {
    //build the decode for the non-user assignable counters
    super.buildDecode()

    //tell the CSR what to do with signal information for the counters
    when(csrFile.csr_wen) {
      for (((e, c), i) <- (reg_eventStorage zip reg_counterStorage) zipWithIndex) {
        writeCounter(i + firstMHPC, c, csrFile.wdata)
        when (csrFile.decoded_addr(i + firstHPE)) { e := perfEventSets.maskEventSelector(csrFile.wdata) }
      }
    }

    perfEventSets.print()
  }

  def writeCounter(lo: Int, ctr: UInt, wdata: UInt) = {
    if (csrFile.xLen == 32) {
      val hi = lo + CSRs.mcycleh - CSRs.mcycle
      when (csrFile.decoded_addr(lo)) { ctr := Cat(ctr(ctr.getWidth-1, 32), wdata) }
      when (csrFile.decoded_addr(hi)) { ctr := Cat(wdata(ctr.getWidth-33, 0), ctr(31, 0)) }
    } else {
      when (csrFile.decoded_addr(lo)) { ctr := wdata(ctr.getWidth-1, 0) }
    }
  }
}
