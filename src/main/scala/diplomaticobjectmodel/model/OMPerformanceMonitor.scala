// See LICENSE.SiFive for license details.

package freechips.rocketchip.diplomaticobjectmodel.model

import freechips.rocketchip.rocket.RocketCoreParams

case class OMPerformanceMonitor(
  specifications: List[OMSpecification],
  hasBasicCounters: Boolean,
  nAdditionalCounters: Int,
  _types: Seq[String] = Seq("OMPerformanceMonitor", "OMComponent", "OMCompoundType")
) extends OMComponent

object PerformanceMonitor {
  //this is never called
  def perfmon(coreParams: RocketCoreParams): Option[OMPerformanceMonitor] = {
    /*if (true || 28 > 0) {
      Some(OMPerformanceMonitor(
        specifications = List[OMSpecification](PrivilegedArchitectureExtensions.specVersion(MachineLevelISA, "1.10")),
        hasBasicCounters = true, //coreParams.haveBasicCounters,
        nAdditionalCounters = 28//coreParams.nPerfCounters
      ))
    }
    else { */
      None
    // }
  }
}
