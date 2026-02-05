package dev.quiteboring.swift.finder.movement

import it.unimi.dsi.fastutil.objects.Object2ByteOpenHashMap
import net.minecraft.block.BlockState
import net.minecraft.block.Blocks
import net.minecraft.block.CarpetBlock
import net.minecraft.util.math.BlockPos

object MovementHelper {

  private const val UNKNOWN: Byte = -1
  private const val SOLID: Byte = 1
  private const val PASSABLE: Byte = 0

  private val solidityCache = Object2ByteOpenHashMap<BlockState>().apply {
    defaultReturnValue(UNKNOWN)
  }

  private val ALWAYS_PASSABLE = setOf(
    Blocks.AIR, Blocks.CAVE_AIR, Blocks.VOID_AIR,
    Blocks.TALL_GRASS, Blocks.SHORT_GRASS, Blocks.FERN, Blocks.LARGE_FERN,
    Blocks.DEAD_BUSH, Blocks.DANDELION, Blocks.POPPY,
    Blocks.TORCH, Blocks.WALL_TORCH, Blocks.REDSTONE_TORCH,
    Blocks.SOUL_TORCH, Blocks.SOUL_WALL_TORCH,
    Blocks.RAIL, Blocks.POWERED_RAIL, Blocks.DETECTOR_RAIL, Blocks.ACTIVATOR_RAIL,
    Blocks.REDSTONE_WIRE, Blocks.LEVER, Blocks.TRIPWIRE, Blocks.TRIPWIRE_HOOK
  )

  private val ALWAYS_SOLID = setOf(
    Blocks.STONE, Blocks.GRANITE, Blocks.DIORITE, Blocks.ANDESITE,
    Blocks.DIRT, Blocks.GRASS_BLOCK, Blocks.PODZOL, Blocks.MYCELIUM,
    Blocks.COBBLESTONE, Blocks.MOSSY_COBBLESTONE,
    Blocks.OAK_PLANKS, Blocks.SPRUCE_PLANKS, Blocks.BIRCH_PLANKS,
    Blocks.JUNGLE_PLANKS, Blocks.ACACIA_PLANKS, Blocks.DARK_OAK_PLANKS,
    Blocks.NETHERRACK, Blocks.END_STONE, Blocks.OBSIDIAN, Blocks.BEDROCK,
    Blocks.SAND, Blocks.RED_SAND, Blocks.GRAVEL, Blocks.CLAY,
    Blocks.BRICKS, Blocks.STONE_BRICKS, Blocks.DEEPSLATE
  )

  fun isSolid(ctx: CalculationContext, x: Int, y: Int, z: Int): Boolean {
    val state = ctx.get(x, y, z) ?: return false
    return isSolidState(ctx, state, x, y, z)
  }

  fun isSolidState(ctx: CalculationContext, state: BlockState, x: Int, y: Int, z: Int): Boolean {
    if (state.isAir) return false

    val block = state.block

    if (block is CarpetBlock) return false

    if (ALWAYS_PASSABLE.contains(block)) return false
    if (ALWAYS_SOLID.contains(block)) return true

    val cached = solidityCache.getByte(state)
    if (cached != UNKNOWN) {
      return cached == SOLID
    }

    ctx.bsa.mutablePos.set(x, y, z)
    val solid = !state.getCollisionShape(ctx.world, ctx.bsa.mutablePos).isEmpty
    solidityCache.put(state, if (solid) SOLID else PASSABLE)
    return solid
  }

  fun isPassable(ctx: CalculationContext, x: Int, y: Int, z: Int): Boolean {
    val state = ctx.get(x, y, z) ?: return false
    return !isSolidState(ctx, state, x, y, z)
  }

  fun isSafe(ctx: CalculationContext, x: Int, y: Int, z: Int): Boolean {
    if (!hasValidGround(ctx, x, y - 1, z)) return false
    if (!isPassable(ctx, x, y, z)) return false
    if (!isPassable(ctx, x, y + 1, z)) return false
    return true
  }

  private fun hasValidGround(ctx: CalculationContext, x: Int, y: Int, z: Int): Boolean {
    val state = ctx.get(x, y, z) ?: return false

    if (state.block is CarpetBlock) {
      return isSolid(ctx, x, y - 1, z)
    }

    return isSolidState(ctx, state, x, y, z)
  }

  // Convenience overloads that accept BlockPos to reduce call-site boilerplate
  fun isSolid(ctx: CalculationContext, pos: BlockPos): Boolean = isSolid(ctx, pos.x, pos.y, pos.z)
  fun isPassable(ctx: CalculationContext, pos: BlockPos): Boolean = isPassable(ctx, pos.x, pos.y, pos.z)
  fun isSafe(ctx: CalculationContext, pos: BlockPos): Boolean = isSafe(ctx, pos.x, pos.y, pos.z)

}
