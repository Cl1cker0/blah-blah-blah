package dev.quiteboring.swift.finder.costs

class ActionCosts(
  jumpBoostAmplifier: Int = -1
) {

  val INF_COST = 1e6
  val SPRINT_ONE_BLOCK_TIME = 1.0 / 0.2806
  val SPRINT_DIAGONAL_TIME = SPRINT_ONE_BLOCK_TIME * 1.414
  val JUMP_UP_ONE_BLOCK_TIME: Double
  val MOMENTUM_LOSS_PENALTY = 6.0
  val JUMP_PENALTY = 2.0
  val GAP_JUMP_REWARD_OFFSET = 1.5
  val SLAB_ASCENT_TIME = SPRINT_ONE_BLOCK_TIME * 1.1
  val WALK_OFF_EDGE_TIME = SPRINT_ONE_BLOCK_TIME * 0.5
  val LAND_RECOVERY_TIME = 2.0

  private val fallTimes: DoubleArray = generateFallTimes()

  init {
    var vel = 0.42 + (jumpBoostAmplifier + 1).coerceAtLeast(0) * 0.1
    var jumpTicks = 0.0

    while (vel > 0) {
      vel = (vel - 0.08) * 0.98
      jumpTicks++
    }

    JUMP_UP_ONE_BLOCK_TIME = jumpTicks + MOMENTUM_LOSS_PENALTY + SPRINT_ONE_BLOCK_TIME
  }

  private fun generateFallTimes(): DoubleArray {
    val times = DoubleArray(257)
    var currentDistance = 0.0
    var tick = 0
    var velocity = 0.0

    for (targetDistance in 1..256) {
      while (currentDistance < targetDistance) {
        velocity = (velocity - 0.08) * 0.98
        currentDistance -= velocity
        tick++
      }
      times[targetDistance] = tick.toDouble()
    }

    return times
  }

  fun getFallTime(blocks: Int): Double {
    if (blocks <= 0) return 0.0
    if (blocks >= fallTimes.size) return INF_COST
    return fallTimes[blocks] + LAND_RECOVERY_TIME
  }

}
