package dev.quiteboring.swift.finder.movement.movements

import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.Movement
import dev.quiteboring.swift.finder.movement.MovementHelper
import dev.quiteboring.swift.finder.movement.MovementResult
import net.minecraft.util.math.BlockPos

class MovementDescend(from: BlockPos, to: BlockPos) : Movement(from, to) {

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
      if (!MovementHelper.isPassable(ctx, destX, y, destZ)) return
      if (!MovementHelper.isPassable(ctx, destX, y + 1, destZ)) return

      for (fallDist in 1..ctx.maxFallHeight) {
        val destY = y - fallDist

        if (!MovementHelper.isPassable(ctx, destX, destY + 1, destZ)) return
        if (!MovementHelper.isPassable(ctx, destX, destY, destZ)) return

        if (MovementHelper.isSolid(ctx, destX, destY - 1, destZ)) {
          res.set(destX, destY, destZ)

          var cost = ctx.cost.WALK_OFF_EDGE_TIME + ctx.cost.getFallTime(fallDist)

          if (fallDist > 3) {
            cost += (fallDist - 3) * (fallDist - 3) * 2.0
          }

          cost += ctx.wdc.getPathPenalty(destX, destY, destZ)
          res.cost = cost

          return
        }
      }
    }
  }

}
