package freechips.rocketchip.util

import chisel3._
import scala.collection.mutable._

object SignalThreadder {

  class SignalPassthrough extends Module {
    val io = IO(new Bundle {
      val signalIn = Input(Bool())
      val signalOut = Output(Bool())
    })

    io.signalOut := io.signalIn
  }

  val signals = HashMap[String, SignalPassthrough]()

  //pluck the signal from the context and thread it through a module
  def pluck(name: String, signal: Data): Data = {
    val newPassthrough = Module(new SignalPassthrough)

    newPassthrough.io.signalIn := signal

    signals += (name -> newPassthrough)

    newPassthrough.io.signalOut
  }

  //get signal by name
  def thread(name: String) = signals(name).io.signalOut
}
