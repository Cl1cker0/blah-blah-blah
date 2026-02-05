package dev.quiteboring.swift.command

import com.mojang.brigadier.arguments.IntegerArgumentType
import dev.quiteboring.swift.event.Context
import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.finder.movement.MovementHelper
import dev.quiteboring.swift.util.PlayerUtils
import dev.quiteboring.swift.util.render.drawBox
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback
import net.minecraft.client.MinecraftClient
import net.minecraft.util.math.BlockPos
import net.minecraft.util.math.Box
import java.awt.Color
import kotlin.math.abs

object HeatmapCommand {

  private var boxes: List<Pair<Box, Color>>? = null
  private var mode = Mode.PENALTY
  private var cachedCenter: BlockPos? = null
  private var cachedRadius = 0

  private const val ALPHA = 100

  enum class Mode { PENALTY, EDGE, WALL }

  fun dispatch() {
    ClientCommandRegistrationCallback.EVENT.register { dispatcher, _ ->
      dispatcher.register(
        ClientCommandManager.literal("heatmap")
          .then(
            ClientCommandManager.argument("radius", IntegerArgumentType.integer(1, 64))
              .executes { ctx ->
                val radius = IntegerArgumentType.getInteger(ctx, "radius")
                val pos = PlayerUtils.getBlockStandingOn() ?: return@executes 1
                cachedCenter = pos
                cachedRadius = radius
                generate()
                1
              }
          )
          .then(ClientCommandManager.literal("clear").executes {
            boxes = null
            cachedCenter = null
            cachedRadius = 0
            1
          })
          .then(ClientCommandManager.literal("penalty").executes { setMode(Mode.PENALTY); 1 })
          .then(ClientCommandManager.literal("edge").executes { setMode(Mode.EDGE); 1 })
          .then(ClientCommandManager.literal("wall").executes { setMode(Mode.WALL); 1 })
          .then(ClientCommandManager.literal("inspect").executes { inspect(); 1 })
      )
    }
  }

  private fun setMode(newMode: Mode) {
    mode = newMode
    println("Mode: $mode")

    if (cachedCenter != null) {
      generate()
    }
  }

  private fun inspect() {
    val pos = PlayerUtils.getBlockStandingOn() ?: return
    val ctx = CalculationContext()
    val x = pos.x
    val y = pos.y + 1
    val z = pos.z

    println("Inspecting: ($x, $y, $z)")
    println("Edge: ${ctx.wdc.getEdgeDistance(x, y, z)}")
    println("Wall: ${ctx.wdc.getWallDistance(x, y, z)}")
    println("Penalty: ${ctx.wdc.getPathPenalty(x, y, z)}")
  }

  private fun generate() {
    val center = cachedCenter ?: return
    val radius = cachedRadius
    val ctx = CalculationContext()
    val world = MinecraftClient.getInstance().world ?: return
    val result = mutableListOf<Pair<Box, Color>>()

    val radiusSq = radius * radius
    val searchOrder = (-10..10).sortedBy { abs(it) }

    for (dx in -radius..radius) {
      for (dz in -radius..radius) {
        if (dx * dx + dz * dz > radiusSq) continue

        val x = center.x + dx
        val z = center.z + dz

        for (dy in searchOrder) {
          val y = center.y + dy + 1

          if (MovementHelper.isSafe(ctx, x, y, z)) {
            val blockPos = BlockPos(x, y - 1, z)
            val shape = world.getBlockState(blockPos).getOutlineShape(world, blockPos)

            val box = if (!shape.isEmpty) {
              shape.boundingBox.offset(x.toDouble(), (y - 1).toDouble(), z.toDouble())
            } else {
              Box(x.toDouble(), (y - 1).toDouble(), z.toDouble(), x + 1.0, y.toDouble(), z + 1.0)
            }

            val color = when (mode) {
              Mode.PENALTY -> penaltyColor(ctx.wdc.getPathPenalty(x, y, z))
              Mode.EDGE -> distanceColor(ctx.wdc.getEdgeDistance(x, y, z))
              Mode.WALL -> distanceColor(ctx.wdc.getWallDistance(x, y, z))
            }

            result.add(box to color)
            break
          }
        }
      }
    }

    boxes = result
    println("Heatmap: ${result.size} points")
  }

  private fun penaltyColor(penalty: Double): Color {
    return when {
      penalty <= 1.0 -> Color(0, 255, 0, ALPHA)
      penalty <= 5.0 -> Color(128, 255, 0, ALPHA)
      penalty <= 12.0 -> Color(255, 255, 0, ALPHA)
      penalty <= 25.0 -> Color(255, 128, 0, ALPHA)
      else -> Color(255, 0, 0, ALPHA)
    }
  }

  private fun distanceColor(dist: Int): Color {
    return when (dist) {
      0 -> Color(255, 0, 0, ALPHA)
      1 -> Color(255, 128, 0, ALPHA)
      2 -> Color(255, 255, 0, ALPHA)
      3 -> Color(128, 255, 0, ALPHA)
      else -> Color(0, 255, 0, ALPHA)
    }
  }

  fun onRender(ctx: Context) {
    boxes?.let {
      it.forEach { (box, color) ->
        ctx.drawBox(box, color)
      }
    }
  }

}
