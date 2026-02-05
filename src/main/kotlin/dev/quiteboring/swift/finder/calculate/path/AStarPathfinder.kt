package dev.quiteboring.swift.finder.calculate.path

import dev.quiteboring.swift.finder.calculate.Path
import dev.quiteboring.swift.finder.calculate.PathNode
import dev.quiteboring.swift.finder.calculate.openset.BinaryHeapOpenSet
import dev.quiteboring.swift.finder.goal.IGoal
import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.MovementResult
import dev.quiteboring.swift.finder.movement.Moves
import it.unimi.dsi.fastutil.longs.Long2ObjectOpenHashMap

/**
 * Thank you EpsilonPhoenix for this superb class!
 */
class AStarPathfinder(
  private val startX: Int,
  private val startY: Int,
  private val startZ: Int,
  private val goal: IGoal,
  private val ctx: CalculationContext,
  private val heuristicWeight: Double = 1.1  // weighted A*, change this if it messes up the paths. but it's faster at 1.1 i think.
) {

  private val nodeMap = Long2ObjectOpenHashMap<PathNode>()
  private val moves = Moves.entries.toTypedArray()
  private val res = MovementResult()

  fun findPath(): Path? {
    val openSet = BinaryHeapOpenSet()
    val startNode = getOrCreateNode(startX, startY, startZ)

    startNode.gCost = 0.0
    startNode.fCost = startNode.hCost * heuristicWeight
    openSet.add(startNode)

    val startTime = System.currentTimeMillis()

    while (!openSet.isEmpty()) {
      val current = openSet.poll()
      val cx = current.x
      val cy = current.y
      val cz = current.z

      if (goal.isAtGoal(cx, cy, cz)) {
        val elapsed = System.currentTimeMillis() - startTime
        return Path(current, elapsed)
      }

      val currentCost = current.gCost

      for (move in moves) {
        res.reset()
        move.calculate(ctx, cx, cy, cz, res)

        val moveCost = res.cost
        if (moveCost >= ctx.cost.INF_COST) continue

        val nx = res.x
        val ny = res.y
        val nz = res.z

        val neighbour = getOrCreateNode(nx, ny, nz)
        val newCost = currentCost + moveCost

        if (newCost < neighbour.gCost) {
          neighbour.parent = current
          neighbour.gCost = newCost
          neighbour.fCost = newCost + neighbour.hCost * heuristicWeight

          if (neighbour.heapPosition == -1) {
            openSet.add(neighbour)
          } else {
            openSet.relocate(neighbour)
          }
        }
      }
    }

    return null
  }

  private fun getOrCreateNode(x: Int, y: Int, z: Int): PathNode {
    val key = PathNode.coordKey(x, y, z)
    var node = nodeMap.get(key)

    if (node == null) {
      node = PathNode(x, y, z, goal)
      nodeMap.put(key, node)
    }

    return node
  }

}
