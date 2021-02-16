// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.rocket

import Chisel._
import Chisel.ImplicitConversions._
import freechips.rocketchip.util._
import freechips.rocketchip.util.property._
import freechips.rocketchip.tile._
import freechips.rocketchip.config.Parameters
import chisel3.{withClock}
import scala.collection.mutable.ArrayBuffer

class PerfCounterIO(implicit p: Parameters) extends CoreBundle
    with HasCoreParameters {
  val eventSel = UInt(OUTPUT, xLen)
  val inc = UInt(INPUT, log2Ceil(1+retireWidth))
}

class PerformanceCounters(perfEventSets: EventSets = new EventSets(),
  csrFile: CSRFile) {

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

  val delegable_counters = ((BigInt(1) << (csrFile.nPerfCounters + firstHPM)) - 1).U
  val (reg_mcounteren, read_mcounteren) = {
    val reg = Reg(UInt(32.W))
    (reg, Mux(csrFile.usingUser, reg & delegable_counters, 0.U))
  }
  val (reg_scounteren, read_scounteren) = {
    val reg = Reg(UInt(32.W))
    (reg, Mux(csrFile.usingSupervisor, reg & delegable_counters, 0.U))
  }

  val reg_mcountinhibit = RegInit(0.U((firstHPM + csrFile.nPerfCounters).W))
  csrFile.io.inhibit_cycle := reg_mcountinhibit(0)
  val reg_instret = WideCounter(64, csrFile.io.retire, inhibit = reg_mcountinhibit(2))
  val reg_cycle = if (csrFile.enableCommitLog) WideCounter(64, csrFile.io.retire, inhibit = reg_mcountinhibit(0))
    else withClock(csrFile.io.ungated_clock) { WideCounter(64, !csrFile.io.csr_stall, inhibit = reg_mcountinhibit(0)) }
  csrFile.io.time := reg_cycle
  val reg_hpmevent = csrFile.io.counters.map(c => Reg(init = UInt(0, csrFile.xLen)))
    (csrFile.io.counters zip reg_hpmevent) foreach { case (c, e) => c.eventSel := e }
  val reg_hpmcounter = csrFile.io.counters.zipWithIndex.map { case (c, i) =>
    WideCounter(hpmWidth, c.inc, reset = false, inhibit = reg_mcountinhibit(firstHPM+i)) }

  def buildMappings() = {
    if (csrFile.coreParams.haveBasicCounters) {
      csrFile.read_mapping += CSRs.mcountinhibit -> reg_mcountinhibit
      csrFile.read_mapping += CSRs.mcycle -> reg_cycle
      csrFile.read_mapping += CSRs.minstret -> reg_instret

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

      if (csrFile.usingUser) {
        csrFile.read_mapping += CSRs.mcounteren -> read_mcounteren
        csrFile.read_mapping += CSRs.cycle -> reg_cycle
        csrFile.read_mapping += CSRs.instret -> reg_instret
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

    if (csrFile.usingSupervisor) {
      csrFile.read_mapping += CSRs.scounteren -> read_scounteren
    }
  }

  def buildDecode() = {
    when(csrFile.csr_wen) {
      for (((e, c), i) <- (reg_hpmevent zip reg_hpmcounter) zipWithIndex) {
        writeCounter(i + firstMHPC, c, csrFile.wdata)
        when (csrFile.decoded_addr(i + firstHPE)) { e := perfEventSets.maskEventSelector(csrFile.wdata) }
      }
      if (csrFile.coreParams.haveBasicCounters) {
        when (csrFile.decoded_addr(CSRs.mcountinhibit)) { reg_mcountinhibit := csrFile.wdata & ~2.U(csrFile.xLen.W) }  // mcountinhibit bit [1] is tied zero
        writeCounter(CSRs.mcycle, reg_cycle, csrFile.wdata)
        writeCounter(CSRs.minstret, reg_instret, csrFile.wdata)
      }
    }

    for (io_dec <- csrFile.io.decode) {
      val counter_addr = io_dec.csr(log2Ceil(read_mcounteren.getWidth)-1, 0)
      val allow_counter = (csrFile.reg_mstatus.prv > PRV.S || read_mcounteren(counter_addr)) &&
        (!csrFile.usingSupervisor || csrFile.reg_mstatus.prv >= PRV.S || read_scounteren(counter_addr))

      when((io_dec.csr.inRange(firstCtr, firstCtr + CSR.nCtr) || io_dec.csr.inRange(firstCtrH, firstCtrH + CSR.nCtr))
        && !allow_counter) {
        io_dec.read_illegal := false.B
      }
    }

    if (csrFile.usingSupervisor) {
      when (csrFile.decoded_addr(CSRs.scounteren)) { reg_scounteren := csrFile.wdata }
    }

    if (csrFile.usingUser) {
      when (csrFile.decoded_addr(CSRs.mcounteren)) { reg_mcounteren := csrFile.wdata }
    }

    perfEventSets.print()
  }

  private def writeCounter(lo: Int, ctr: WideCounter, wdata: UInt) = {
    if (csrFile.xLen == 32) {
      val hi = lo + CSRs.mcycleh - CSRs.mcycle
      when (csrFile.decoded_addr(lo)) { ctr := Cat(ctr(ctr.getWidth-1, 32), wdata) }
      when (csrFile.decoded_addr(hi)) { ctr := Cat(wdata(ctr.getWidth-33, 0), ctr(31, 0)) }
    } else {
      when (csrFile.decoded_addr(lo)) { ctr := wdata(ctr.getWidth-1, 0) }
    }
  }
}

case class Event(name: String, signal: () => Bool, offset: Int)

class EventSet(val gate: (UInt, UInt) => Bool, numEvents: Int) {
  val events = ArrayBuffer[Event]()
  val hits = Wire(Vec(numEvents, Bool()))
  def check(mask: UInt) = {
    hits := events.map(_.signal())
    gate(mask, hits.asUInt)
  }
  def dump(): Unit = {
    for (e <- events)
      when (check(1.U << e.offset)) { printf(s"Event $e.name\n") }
  }
  def withCovers: Unit = events.foreach {
    e => cover(gate((1.U << e.offset), (e.signal() << e.offset)), e.name)
  }

  def addEvent(name: String, signal: () => Bool, offset: Int) {
    val newEvent = new Event(name, signal, offset)
    events.foreach {
      e => {
        if (newEvent.name == e.name) throw new Exception("Event name " + e.name + " is already used.")
        if (newEvent.offset == e.offset) throw new Exception("Event " + newEvent.name + " offset already used.")
      }
    }
    events += newEvent
  }

  def print() = {
    events.foreach {e => println("Event Name: " + e.name + ", Bit Number: " + e.offset)}
  }

  def size() = events.size
}

class EventSets() {
  val eventSets = ArrayBuffer[EventSet]()

  def maskEventSelector(eventSel: UInt): UInt = {
    // allow full associativity between counters and event sets (for now?)
    val setMask = (BigInt(1) << eventSetIdBits) - 1
    val maskMask = ((BigInt(1) << eventSets.map(_.size).max) - 1) << maxEventSetIdBits
    eventSel & (setMask | maskMask).U
  }

  private def decode(counter: UInt): (UInt, UInt) = {
    require(eventSets.size <= (1 << maxEventSetIdBits))
    require(eventSetIdBits > 0)
    (counter(eventSetIdBits-1, 0), counter >> maxEventSetIdBits)
  }

  def evaluate(eventSel: UInt): Bool = {
    val (set, mask) = decode(eventSel)
    //the set ID is determined by the order in which it appears in eventSets
    val sets = for (e <- eventSets) yield {
      require(e.hits.getWidth <= mask.getWidth, s"too many events ${e.hits.getWidth} wider than mask ${mask.getWidth}")
      e check mask
    }
    sets(set)
  }

  def addEventSet(newEventSet: EventSet) {
    eventSets += newEventSet
  }

  def cover() = eventSets.foreach { _ withCovers }

  def print() = {
    println("Configured Performance Counter Events:")
    for((e,i) <- eventSets.zipWithIndex) {
      println("Event Set " + i)
      e.print
    }
  }

  private def eventSetIdBits = {
    val bits = log2Ceil(eventSets.size)
    //require(bits <= maxEventSetIdBits)
    bits
  }
  private def maxEventSetIdBits = 8
}

class SuperscalarEventSets(val eventSets: Seq[(Seq[EventSet], (UInt, UInt) => UInt)]) {
  def evaluate(eventSel: UInt): UInt = {
    val (set, mask) = decode(eventSel)
    val sets = for ((sets, reducer) <- eventSets) yield {
      sets.map { set =>
        require(set.hits.getWidth <= mask.getWidth, s"too many events ${set.hits.getWidth} wider than mask ${mask.getWidth}")
        set.check(mask)
      }.reduce(reducer)
    }
    val zeroPadded = sets.padTo(1 << eventSetIdBits, 0.U)
    zeroPadded(set)
  }

  //TODO: Fix this
  def toScalarEventSets: EventSets = new EventSets()

  def cover(): Unit = { eventSets.foreach(_._1.foreach(_.withCovers)) }

  private def decode(counter: UInt): (UInt, UInt) = {
    require(eventSets.size <= (1 << maxEventSetIdBits))
    require(eventSetIdBits > 0)
    (counter(eventSetIdBits-1, 0), counter >> maxEventSetIdBits)
  }

  private def eventSetIdBits = log2Ceil(eventSets.size)
  private def maxEventSetIdBits = 8

  require(eventSets.forall(s => s._1.forall(_.size == s._1.head.size)))
  require(eventSetIdBits <= maxEventSetIdBits)
}
