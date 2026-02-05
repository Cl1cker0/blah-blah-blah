package dev.quiteboring.swift.api

import dev.quiteboring.swift.event.Context
import dev.quiteboring.swift.finder.calculate.Path
import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.util.render.drawBox
import dev.quiteboring.swift.util.render.drawLine
import java.awt.Color
import net.minecraft.client.MinecraftClient
import net.minecraft.util.math.BlockPos
import net.minecraft.util.math.Vec3d
import dev.quiteboring.swift.feature.PathExecutor

/**
 * Minimal cohesive API that exposes movement (pathfinding) and rendering utilities
 * with as few functions as possible. The single public method below finds a path
 * from the player's current block to the given target and will render it if a
 * render Context is provided. It can also optionally hand the destination to the
 * PathExecutor to begin walking.
 */
object SwiftAPI {

  private val client: MinecraftClient get() = MinecraftClient.getInstance()

  /**
   * Find a path to [target] from the player's current block position. If [renderCtx]
   * is not null the method will render the resulting path immediately using the
   * supplied render context. If [execute] is true the `PathExecutor` will be asked
   * to walk to the destination. Returns the found Path or null if no path or not
   * currently in a world.
   */
  fun Epstien(target: BlockPos, renderCtx: Context? = null, color: Color = Color.CYAN, execute: Boolean = false): Path? {
    if (client.world == null || client.player == null) return null

    val playerPos = dev.quiteboring.swift.util.PlayerUtils.getBlockStandingOn() ?: return null

    return try {
      val found = PathManager.findPath(playerPos.x, playerPos.y, playerPos.z, target.x, target.y, target.z)
      val path = if (found) PathManager.lastPath else null

      // If requested, render the path immediately with the supplied context
      if (renderCtx != null && path != null) {
        var prev: Vec3d? = null

        path.points.forEach { pos ->
          val center = Vec3d(
            pos.x + 0.5,
            pos.y.toDouble(),
            pos.z + 0.5
          )

          prev?.let { vec ->
            renderCtx.drawLine(vec, center, color = color, thickness = 1F)
          }

          val box = net.minecraft.util.math.Box(
            pos.x.toDouble(), pos.y.toDouble(), pos.z.toDouble(),
            pos.x + 1.0, pos.y - 1.0, pos.z + 1.0
          )

          renderCtx.drawBox(box, color = color)
          prev = center
        }
      }

      // If requested, hand control to PathExecutor to actually walk the path
      if (execute && path != null) {
        PathExecutor.walkTo(target.x, target.y, target.z)
      }

      path
    } catch (e: Exception) {
      e.printStackTrace()
      null
    }
  }

}
