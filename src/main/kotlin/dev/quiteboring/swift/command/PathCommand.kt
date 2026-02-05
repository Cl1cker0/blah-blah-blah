package dev.quiteboring.swift.command

import com.mojang.brigadier.arguments.IntegerArgumentType
import dev.quiteboring.swift.finder.calculate.Path
import dev.quiteboring.swift.finder.calculate.path.AStarPathfinder
import dev.quiteboring.swift.event.Context
import dev.quiteboring.swift.finder.goal.Goal
import dev.quiteboring.swift.finder.movement.CalculationContext
import dev.quiteboring.swift.util.PlayerUtils
import dev.quiteboring.swift.util.render.drawBox
import dev.quiteboring.swift.util.render.drawLine
import java.awt.Color
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback
import net.minecraft.client.MinecraftClient
import net.minecraft.text.Text
import net.minecraft.util.math.Box
import net.minecraft.util.math.Vec3d

object PathCommand {

  private var path: Path? = null

  fun dispatch() {
    ClientCommandRegistrationCallback.EVENT.register { dispatcher, _ ->
      val xArg = ClientCommandManager.argument("x", IntegerArgumentType.integer())
      val yArg = ClientCommandManager.argument("y", IntegerArgumentType.integer())
      val zArg = ClientCommandManager.argument("z", IntegerArgumentType.integer())

      val pathfindCommand = ClientCommandManager.literal("pathfind")
        .then(ClientCommandManager.literal("clear").executes {
          this.path = null
          MinecraftClient.getInstance().inGameHud.chatHud.addMessage(Text.of("Rendering Cleared"))
          1
        })
        .then(xArg.then(yArg.then(zArg.executes { context ->
          val x = IntegerArgumentType.getInteger(context, "x")
          val y = IntegerArgumentType.getInteger(context, "y")
          val z = IntegerArgumentType.getInteger(context, "z")

          val player = PlayerUtils.getBlockStandingOn() ?: return@executes 1
          val ctx = CalculationContext()

          AStarPathfinder(
            player.x, player.y, player.z,
            Goal(x, y, z, ctx), ctx
          ).findPath()?.let {
            this.path = it
            MinecraftClient.getInstance().inGameHud.chatHud.addMessage(Text.of("${it.timeTaken} ms"))
          }

          return@executes 1
        })))

      dispatcher.register(pathfindCommand)
    }
  }

  fun onRender(ctx: Context) {
    path?.let {
      var prev: Vec3d? = null

      it.points.forEach { pos ->
        val center = Vec3d(
          pos.x + 0.5,
          pos.y.toDouble(),
          pos.z + 0.5
        )

        prev?.let { vec ->
          ctx.drawLine(vec, center, color = Color(255, 132, 94), thickness = 1F)
        }

        val box = Box(
          pos.x.toDouble(), pos.y.toDouble(), pos.z.toDouble(),
          pos.x + 1.0, pos.y - 1.0, pos.z + 1.0
        )

        ctx.drawBox(box, color = Color(255, 132, 94))
        prev = center
      }
    }
  }

}
