package dev.quiteboring.swift.finder.calculate.openset

import dev.quiteboring.swift.finder.calculate.PathNode

class BinaryHeapOpenSet(initialSize: Int = 1024) {

  private var items = arrayOfNulls<PathNode>(initialSize)
  var size = 0
    private set

  fun add(node: PathNode) {
    if (size >= items.size - 1) {
      items = items.copyOf(items.size * 2)
    }

    size++
    node.heapPosition = size
    items[size] = node
    siftUp(node)
  }

  fun relocate(node: PathNode) {
    siftUp(node)
  }

  private fun siftUp(node: PathNode) {
    var pos = node.heapPosition
    val cost = node.fCost

    while (pos > 1) {
      val parentPos = pos ushr 1
      val parent = items[parentPos]!!

      if (cost >= parent.fCost) break

      items[pos] = parent
      parent.heapPosition = pos
      pos = parentPos
    }

    items[pos] = node
    node.heapPosition = pos
  }

  fun poll(): PathNode {
    val result = items[1]!!
    result.heapPosition = -1

    val last = items[size]!!
    items[size] = null
    size--

    if (size > 0) {
      items[1] = last
      last.heapPosition = 1
      siftDown(last)
    }

    return result
  }

  private fun siftDown(node: PathNode) {
    var pos = 1
    val cost = node.fCost
    val halfSize = size ushr 1

    while (pos <= halfSize) {
      var childPos = pos shl 1
      var child = items[childPos]!!

      val rightPos = childPos + 1
      if (rightPos <= size) {
        val right = items[rightPos]!!
        if (right.fCost < child.fCost) {
          childPos = rightPos
          child = right
        }
      }

      if (cost <= child.fCost) break

      items[pos] = child
      child.heapPosition = pos
      pos = childPos
    }

    items[pos] = node
    node.heapPosition = pos
  }

  fun isEmpty() = size == 0
}
