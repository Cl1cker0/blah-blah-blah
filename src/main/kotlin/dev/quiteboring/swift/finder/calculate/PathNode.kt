package dev.quiteboring.swift.finder.calculate

import dev.quiteboring.swift.finder.goal.IGoal

class PathNode(
  val x: Int,
  val y: Int,
  val z: Int,
  goal: IGoal,
) {

  var gCost: Double = 1e6
  val hCost: Double = goal.heuristic(x, y, z)
  var fCost: Double = 1e6
  var heapPosition = -1
  var parent: PathNode? = null

  override fun equals(other: Any?): Boolean {
    if (other !is PathNode) return false
    return other.x == x && other.y == y && other.z == z
  }

  override fun hashCode(): Int = (x * 31 + y) * 31 + z

  companion object {
    @JvmStatic
    fun coordKey(x: Int, y: Int, z: Int): Long {
      val px = (x + 33554432).toLong() and 0x3FFFFFF  // 26 bits
      val py = (y + 2048).toLong() and 0xFFF          // 12 bits
      val pz = (z + 33554432).toLong() and 0x3FFFFFF  // 26 bits
      return (px shl 38) or (py shl 26) or pz
    }
  }
}
