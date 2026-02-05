package dev.quiteboring.swift.finder.movement.movements

import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.Movement
import dev.quiteboring.swift.finder.movement.MovementHelper
import dev.quiteboring.swift.finder.movement.MovementResult
import net.minecraft.util.math.BlockPos

class MovementJumpGap(from: BlockPos, to: BlockPos) : Movement(from, to) {

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

      val dx = destX - x
      val dz = destZ - z
      val dist = if (dx != 0) Math.abs(dx) else Math.abs(dz)
      val dirX = if (dx != 0) dx / dist else 0
      val dirZ = if (dz != 0) dz / dist else 0

      if (!MovementHelper.isPassable(ctx, x, y + 2, z)) return

      for (i in 1 until dist) {
        val checkX = x + (dirX * i)
        val checkZ = z + (dirZ * i)
        if (!MovementHelper.isPassable(ctx, checkX, y, checkZ)) return
        if (!MovementHelper.isPassable(ctx, checkX, y + 1, checkZ)) return
        if (!MovementHelper.isPassable(ctx, checkX, y + 2, checkZ)) return
      }

      res.set(destX, y, destZ)

      var cost = ctx.cost.JUMP_PENALTY + (ctx.cost.SPRINT_ONE_BLOCK_TIME * dist)
      cost += ctx.cost.GAP_JUMP_REWARD_OFFSET

      cost += ctx.wdc.getPathPenalty(destX, y, destZ)
      res.cost = cost
    }
  }
}
