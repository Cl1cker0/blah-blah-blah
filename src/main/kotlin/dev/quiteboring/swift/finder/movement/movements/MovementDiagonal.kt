package dev.quiteboring.swift.finder.movement.movements

import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.Movement
import dev.quiteboring.swift.finder.movement.MovementHelper
import dev.quiteboring.swift.finder.movement.MovementResult
import net.minecraft.util.math.BlockPos

class MovementDiagonal(from: BlockPos, to: BlockPos) : Movement(from, to) {

  override fun calculateCost(ctx: CalculationContext, res: MovementResult) {
    calculateCost(ctx, source.x, source.y, source.z, target.x, target.z, res)
    costs = res.cost
  }

  companion object {
    fun calculateCost(
      ctx: CalculationContext,
      x: Int, y: Int, z: Int,
      destX: Int, destZ: Int,
      res: MovementResult
    ) {
      if (!MovementHelper.isSafe(ctx, destX, y, destZ)) return

      if (MovementHelper.isSolid(ctx, x, y, destZ) || MovementHelper.isSolid(ctx, destX, y, z)) return
      if (MovementHelper.isSolid(ctx, x, y + 1, destZ) || MovementHelper.isSolid(ctx, destX, y + 1, z)) return

      res.set(destX, y, destZ)
      res.cost = ctx.cost.SPRINT_DIAGONAL_TIME + ctx.wdc.getPathPenalty(destX, y, destZ)
    }
  }

}
