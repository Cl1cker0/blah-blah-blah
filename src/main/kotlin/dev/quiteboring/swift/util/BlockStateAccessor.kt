package dev.quiteboring.swift.util

import net.minecraft.block.BlockState
import net.minecraft.block.Blocks
import net.minecraft.util.math.BlockPos
import net.minecraft.world.World
import net.minecraft.world.chunk.Chunk
import net.minecraft.world.chunk.ChunkStatus

class BlockStateAccessor(val world: World) {

  val mutablePos: BlockPos.Mutable = BlockPos.Mutable()

  private companion object {
    const val CACHE_SIZE = 4
    const val CACHE_MASK = CACHE_SIZE - 1
  }

  private val chunks = arrayOfNulls<Chunk>(CACHE_SIZE)
  private val chunkKeys = LongArray(CACHE_SIZE) { Long.MIN_VALUE }
  private var nextSlot = 0

  private val bottomY = world.bottomY
  private val air: BlockState = Blocks.AIR.defaultState

  private fun chunkKey(chunkX: Int, chunkZ: Int): Long =
    (chunkX.toLong() shl 32) or (chunkZ.toLong() and 0xFFFFFFFFL)

  fun get(x: Int, y: Int, z: Int): BlockState? {
    val chunkX = x shr 4
    val chunkZ = z shr 4
    val key = chunkKey(chunkX, chunkZ)

    for (i in 0 until CACHE_SIZE) {
      if (chunkKeys[i] == key) {
        chunks[i]?.let { return getFromChunk(it, x, y, z) }
      }
    }

    val chunk = world.chunkManager.getChunk(chunkX, chunkZ, ChunkStatus.FULL, false)
    if (chunk == null || chunk.sectionArray.all { it.isEmpty }) return null

    chunks[nextSlot] = chunk
    chunkKeys[nextSlot] = key
    nextSlot = (nextSlot + 1) and CACHE_MASK

    return getFromChunk(chunk, x, y, z)
  }

  fun isChunkLoaded(blockX: Int, blockZ: Int): Boolean =
    world.chunkManager.isChunkLoaded(blockX shr 4, blockZ shr 4)

  fun getFromChunk(chunk: Chunk, x: Int, y: Int, z: Int): BlockState {
    val sectionIndex = (y - bottomY) shr 4

    if (sectionIndex < 0 || sectionIndex >= chunk.sectionArray.size) {
      return air
    }

    val section = chunk.sectionArray[sectionIndex]
    if (section.isEmpty) return air

    return section.getBlockState(x and 15, y and 15, z and 15)
  }
}
