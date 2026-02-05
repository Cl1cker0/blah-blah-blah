package dev.quiteboring.swift.finder.movement.movements

import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.Movement
import dev.quiteboring.swift.finder.movement.MovementHelper
import dev.quiteboring.swift.finder.movement.MovementResult
import net.minecraft.block.*
import net.minecraft.util.math.BlockPos

class MovementAscend(from: BlockPos, to: BlockPos) : Movement(from, to) {

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
      if (!MovementHelper.isSafe(ctx, destX, y + 1, destZ)) return
      if (!MovementHelper.isPassable(ctx, x, y + 2, z)) return

      res.set(destX, y + 1, destZ)

      val groundState = ctx.get(destX, y, destZ)
      val block = groundState?.block

      if (block is FenceBlock || block is FenceGateBlock || block is WallBlock) return

      res.cost = if (block is SlabBlock || block is StairsBlock) {
        ctx.cost.SLAB_ASCENT_TIME
      } else {
        ctx.cost.JUMP_UP_ONE_BLOCK_TIME
      } + ctx.wdc.getPathPenalty(destX, y + 1, destZ)
    }
  }

}
