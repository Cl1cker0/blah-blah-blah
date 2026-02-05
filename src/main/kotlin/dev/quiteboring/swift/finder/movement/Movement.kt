package dev.quiteboring.swift.finder.movement

import net.minecraft.util.math.BlockPos

abstract class Movement(val source: BlockPos, val target: BlockPos) {

  var costs: Double = 1e6
  open fun getCost() = costs

  abstract fun calculateCost(ctx: CalculationContext, res: MovementResult)

}
