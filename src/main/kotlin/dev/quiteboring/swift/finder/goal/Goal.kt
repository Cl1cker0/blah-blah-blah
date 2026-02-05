package dev.quiteboring.swift.finder.goal

import dev.quiteboring.swift.finder.movement.CalculationContext
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class Goal(
  val goalX: Int,
  val goalY: Int,
  val goalZ: Int,
  ctx: CalculationContext
) : IGoal {

  // precompute cuz it's slightly faster ig.
  private val sprintCost = ctx.cost.SPRINT_ONE_BLOCK_TIME
  private val diagonalCost = sprintCost * sqrt(2.0)
  private val fallCostPerBlock = ctx.cost.getFallTime(2) * 0.5
  private val jumpCostPerBlock = ctx.cost.JUMP_UP_ONE_BLOCK_TIME * 0.5

  override fun isAtGoal(x: Int, y: Int, z: Int): Boolean {
    return x == goalX && y == goalY && z == goalZ
  }

  override fun heuristic(x: Int, y: Int, z: Int): Double {
    val dx = abs(x - goalX)
    val dy = y - goalY
    val dz = abs(z - goalZ)

    val diag = min(dx, dz)
    val straight = max(dx, dz) - diag
    val horizontal = diag * diagonalCost + straight * sprintCost

    val vertical = when {
      dy > 0 -> dy * fallCostPerBlock
      dy < 0 -> -dy * jumpCostPerBlock
      else -> 0.0
    }

    return horizontal + vertical
  }

}
