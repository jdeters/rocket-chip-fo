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
    val bits = if(eventSets.size == 1) 1 else log2Ceil(eventSets.size)
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
