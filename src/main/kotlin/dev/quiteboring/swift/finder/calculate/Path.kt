package dev.quiteboring.swift.finder.calculate

import net.minecraft.util.math.BlockPos

class Path(endNode: PathNode, val timeTaken: Long) {

  val points: List<BlockPos>
  val keyNodes: List<BlockPos>

  init {
    var curr: PathNode? = endNode
    val list = mutableListOf<BlockPos>()

    while (curr != null) {
      list.addFirst(BlockPos(curr.x, curr.y, curr.z))
      curr = curr.parent
    }

    points = list
    keyNodes = extractKeyPoints(points)
  }

  private fun extractKeyPoints(points: List<BlockPos>): List<BlockPos> {
    if (points.isEmpty()) return emptyList()
    if (points.size <= 2) return points.toList()

    val result = ArrayList<BlockPos>(points.size / 4 + 2)
    result.add(points[0])

    var lastKeyIdx = 0
    var prevDx = points[1].x - points[0].x
    var prevDy = points[1].y - points[0].y
    var prevDz = points[1].z - points[0].z

    for (i in 1 until points.size - 1) {
      val curr = points[i]
      val next = points[i + 1]

      val dx = next.x - curr.x
      val dy = next.y - curr.y
      val dz = next.z - curr.z

      val isVerticalChange =
        (prevDy >= 0 && dy < 0) ||
          (prevDy < 0 && dy >= 0) ||
          (prevDy <= 0 && dy > 0)

      val isTurn = dx != prevDx || dz != prevDz
      val isLongSegment = i - lastKeyIdx >= MAX_SEGMENT_LENGTH

      if (isVerticalChange || isTurn || isLongSegment) {
        result.add(curr)
        lastKeyIdx = i
      }

      prevDx = dx
      prevDy = dy
      prevDz = dz
    }

    result.add(points.last())
    return result
  }

  companion object {
    private const val MAX_SEGMENT_LENGTH = 24
  }

}
