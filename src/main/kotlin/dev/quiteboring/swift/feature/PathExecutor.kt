package dev.quiteboring.swift.feature

import dev.quiteboring.swift.api.PathManager
import dev.quiteboring.swift.util.Rotations
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents
import net.minecraft.client.MinecraftClient
import net.minecraft.client.network.ClientPlayerEntity
import net.minecraft.text.Text
import net.minecraft.util.Formatting
import net.minecraft.util.math.BlockPos
import net.minecraft.util.math.Vec3d
import dev.quiteboring.swift.util.PlayerUtils
import dev.quiteboring.swift.util.AngleUtils
import dev.quiteboring.swift.util.render.drawBox
import dev.quiteboring.swift.util.render.drawLine
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.math.*

/**
 * Kotlin port of PathExecutor adapted to use the mod's existing APIs.
 * - Uses PathManager.lastPath for path data
 * - Uses a small internal Keybinds helper to press/release movement keys
 * - Uses Rotations instance for rotation smoothing
 */
object PathExecutor {
  private val mc: MinecraftClient get() = MinecraftClient.getInstance()
  // private val rotations: Rotations? get() = Rotations.instance

  private val running = AtomicBoolean(false)
  private val shouldWalk = AtomicBoolean(false)

  private var currentPath: List<BlockPos> = emptyList()
  private var smoothPath: List<Vec3d> = emptyList()
  private var currentWaypointIndex = 0

  private var currentLookaheadPoint: Vec3d? = null

  private const val WAYPOINT_REACH_DISTANCE = 1.5
  private const val LOOKAHEAD_DISTANCE = 8.0

  private var tickCounter = 0

  private var wasOnGround = false
  private var wasInWater = false
  private var wasInLava = false
  private var ticksOffGround = 0
  private var ticksInLiquid = 0
  private var lastY = 0.0
  private var stuckTicks = 0
  private var lastPos: Vec3d = Vec3d.ZERO

  private val rotator = QuarticRotator()

  init {
    // Register tick handler
    ClientTickEvents.END_CLIENT_TICK.register(ClientTickEvents.EndTick { client ->
      try {
        if (running.get() && shouldWalk.get()) {
          onTick()
        }
      } catch (e: Exception) {
        e.printStackTrace()
      }
    })
  }

  fun walkTo(x: Int, y: Int, z: Int) {
    shouldWalk.set(true)
    if (mc.world == null || mc.player == null) return
    // Use player's standing block as start
    val start = PlayerUtils.getBlockStandingOn() ?: return
    if (PathManager.findPath(start.x, start.y, start.z, x, y, z)) {
      PathManager.lastPath?.let { p ->
        currentPath = p.points
        startExecution()
        mc.player?.sendMessage(Text.of("[PathExecutor] Walking to $x,$y,$z"), false)
      }
    } else {
      mc.player?.sendMessage(Text.of("[PathExecutor] No path found"), false)
    }
  }

  fun showPathTo(x: Int, y: Int, z: Int) {
    shouldWalk.set(false)
    if (mc.world == null || mc.player == null) return
    val start = PlayerUtils.getBlockStandingOn() ?: return
    if (PathManager.findPath(start.x, start.y, start.z, x, y, z)) {
      mc.player!!.sendMessage(Text.of("[PathExecutor] Path rendered"), false)
    } else {
      mc.player?.sendMessage(Text.of("[PathExecutor] No path found"), false)
    }
  }

  private fun onNewPath() {
    PathManager.lastPath?.let { p ->
      currentPath = p.points
      // if walking, prepare smooth path
      if (shouldWalk.get()) {
        smoothPath = createSmoothPath(currentPath)
        currentWaypointIndex = 0
        tickCounter = 0
        startExecution()
      }
    }
  }

  private fun startExecution() {
    if (running.compareAndSet(false, true)) {
      println("[PathExecutor] Starting execution")
    }
  }

  private fun stopExecution() {
    if (running.compareAndSet(true, false)) {
      Keybinds.releaseAll()
      // rotations?.stopRotate()
      shouldWalk.set(false)
      currentPath = emptyList()
      smoothPath = emptyList()
      currentWaypointIndex = 0
      currentLookaheadPoint = null
      wasOnGround = false
      wasInWater = false
      wasInLava = false
      ticksOffGround = 0
      ticksInLiquid = 0
      stuckTicks = 0
      lastPos = Vec3d.ZERO
      println("[PathExecutor] Stopped execution")
    }
  }

  private fun onTick() {
    val player = mc.player ?: run { stopExecution(); return }
    if (currentPath.isEmpty()) {
      stopExecution()
      return
    }

    tickCounter++
    updateMovementState()

    if (currentWaypointIndex >= currentPath.size) {
      player.sendMessage(Text.of("Destination reached!"), false)
      stopExecution()
      return
    }

    var targetWaypoint = currentPath[currentWaypointIndex]
    // use explicit Vec3d built from player coordinates (avoid private pos field)
    val playerPos = Vec3d(player.x, player.y, player.z)
    val waypointPos = Vec3d(targetWaypoint.x + 0.5, targetWaypoint.y.toDouble(), targetWaypoint.z + 0.5)

    val dx = waypointPos.x - playerPos.x
    val dy = waypointPos.y - playerPos.y
    val dz = waypointPos.z - playerPos.z
    val horizontalDist = sqrt(dx * dx + dz * dz)

    val toWaypoint = Vec3d(dx, 0.0, dz).normalize()
    val lookDir = player.getRotationVector()
    val lookDirHorizontal = Vec3d(lookDir.x, 0.0, lookDir.z).normalize()
    val dotProduct = toWaypoint.dotProduct(lookDirHorizontal)

    val reachedWaypoint = shouldAdvanceWaypoint(horizontalDist, dy, dotProduct)
    if (reachedWaypoint) {
      currentWaypointIndex++
      stuckTicks = 0
      if (currentWaypointIndex >= currentPath.size) {
        player.sendMessage(Text.of("Final destination reached!"), false)
        stopExecution()
        return
      }
      targetWaypoint = currentPath[currentWaypointIndex]
    }

    currentLookaheadPoint = calculateLookaheadPoint(playerPos)

    // Use Quartic Bezier Rotation for smooth tracking
    currentLookaheadPoint?.let { lp ->
      rotator.update(player, lp)
    }

    applyMovement(player, targetWaypoint)
  }

  private fun updateMovementState() {
    val player = mc.player ?: return
    val isOnGround = player.isOnGround
    val isInWater = player.isTouchingWater
    val isInLava = player.isInLava
    // use explicit Vec3d instead of getPos()
    val currentPos = Vec3d(player.x, player.y, player.z)
    val currentY = player.y

    if (!isOnGround) ticksOffGround++ else ticksOffGround = 0
    if (isInWater || isInLava) ticksInLiquid++ else ticksInLiquid = 0

    val horizontalMovement = sqrt((currentPos.x - lastPos.x).pow(2) + (currentPos.z - lastPos.z).pow(2))

    if (horizontalMovement < 0.01 && !isOnGround && ticksOffGround > 5) {
      stuckTicks++
    } else if (horizontalMovement > 0.01) {
      stuckTicks = 0
    }

    wasOnGround = isOnGround
    wasInWater = isInWater
    wasInLava = isInLava
    lastY = currentY
    lastPos = currentPos
  }

  private fun shouldAdvanceWaypoint(horizontalDist: Double, dy: Double, dotProduct: Double): Boolean {
    val player = mc.player ?: return false
    if (horizontalDist < WAYPOINT_REACH_DISTANCE) return true
    if (dotProduct < -0.3) return true

    if (ticksOffGround > 3 && !player.isOnGround && player.velocity.y < -0.1) {
      if (dy <= 0.0 && horizontalDist < WAYPOINT_REACH_DISTANCE * 2.0) return true
    }

    if (ticksInLiquid > 2 && (player.isTouchingWater || player.isInLava)) {
      if (horizontalDist < WAYPOINT_REACH_DISTANCE * 1.5) return true
      if (abs(dy) < 1.5 && horizontalDist < WAYPOINT_REACH_DISTANCE * 2.0) return true
    }

    if (ticksOffGround > 0 && ticksOffGround < 10 && player.velocity.y > 0) {
      if (dy > 0 && horizontalDist < WAYPOINT_REACH_DISTANCE * 1.3) return true
    }

    if (stuckTicks > 30) {
      println("[PathExecutor] Stuck detected, advancing waypoint")
      return true
    }

    return false
  }

  private fun createSmoothPath(path: List<BlockPos>): List<Vec3d> {
    val smoothPoints = ArrayList<Vec3d>()
    for (i in 0 until path.size - 1) {
      val from = path[i]
      val to = path[i + 1]
      val fromVec = Vec3d(from.x + 0.5, from.y.toDouble(), from.z + 0.5)
      val toVec = Vec3d(to.x + 0.5, to.y.toDouble(), to.z + 0.5)
      val distance = fromVec.distanceTo(toVec)
      val steps = ceil(distance).toInt()
      for (step in 0 until steps) {
        val t = step.toDouble() / steps
        val interpolated = Vec3d(
          fromVec.x + (toVec.x - fromVec.x) * t,
          fromVec.y + (toVec.y - fromVec.y) * t,
          fromVec.z + (toVec.z - fromVec.z) * t
        )
        smoothPoints.add(interpolated)
      }
    }
    if (path.isNotEmpty()) {
      val last = path[path.size - 1]
      smoothPoints.add(Vec3d(last.x + 0.5, last.y.toDouble(), last.z + 0.5))
    }
    return smoothPoints
  }

  private fun calculateLookaheadPoint(playerPos: Vec3d): Vec3d {
    if (smoothPath.isEmpty()) return playerPos
    var closestIndex = 0
    var closestDist = Double.MAX_VALUE
    for (i in smoothPath.indices) {
      val dist = playerPos.distanceTo(smoothPath[i])
      if (dist < closestDist) {
        closestDist = dist
        closestIndex = i
      }
    }
    var distanceAhead = 0.0
    for (i in closestIndex until smoothPath.size - 1) {
      val current = smoothPath[i]
      val next = smoothPath[i + 1]
      distanceAhead += current.distanceTo(next)
      if (distanceAhead >= LOOKAHEAD_DISTANCE) {
        val lookahead = smoothPath[i + 1]
        var yOffset = 1.5
        if (i + 2 < smoothPath.size) {
          val nextNext = smoothPath[i + 2]
          val yChange = nextNext.y - lookahead.y
          if (yChange > 0.5) {
            yOffset = 2.5 + yChange * 0.8
          } else if (yChange < -0.5) {
            yOffset = 0.8 + yChange * 0.2
          }
        }
        return Vec3d(lookahead.x, lookahead.y + yOffset, lookahead.z)
      }
    }
    val end = smoothPath.last()
    return Vec3d(end.x, end.y + 1.5, end.z)
  }

  private fun applyMovement(player: ClientPlayerEntity, target: BlockPos) {
    // use player's explicit coords
    val playerPos = Vec3d(player.x, player.y, player.z)
    val targetVec = Vec3d(target.x + 0.5, target.y + 0.5, target.z + 0.5)
    val dx = targetVec.x - playerPos.x
    val dz = targetVec.z - playerPos.z
    val dy = targetVec.y - playerPos.y
    val horizontalDistance = sqrt(dx * dx + dz * dz)
    val targetDir = Vec3d(dx, 0.0, dz).normalize()
    val lookDir = player.getRotationVector()
    val lookDirHorizontal = Vec3d(lookDir.x, 0.0, lookDir.z).normalize()
    val alignment = targetDir.dotProduct(lookDirHorizontal)
    val shouldMove = horizontalDistance > 0.3 || alignment > -0.5
    val isSwimming = player.isTouchingWater || player.isInLava
    val isFalling = !player.isOnGround && player.velocity.y < -0.1 && ticksOffGround > 3
    val isJumping = !player.isOnGround && player.velocity.y > 0 && ticksOffGround < 10

    if (horizontalDistance > 0.1 && shouldMove) {
      if (!isWallAhead(player, targetDir)) {
        Keybinds.holdForward(true)
        if (isSwimming) {
          Keybinds.holdForward(true)
          if (dy > 0.5) Keybinds.holdJump(true) else Keybinds.holdJump(false)
          Keybinds.holdSprint(false)
        } else if (isFalling) {
          Keybinds.holdForward(true)
          Keybinds.holdJump(false)
          if (alignment > 0.7) Keybinds.holdSprint(true) else Keybinds.holdSprint(false)
        } else if (isJumping) {
          Keybinds.holdForward(true)
          Keybinds.holdJump(false)
          Keybinds.holdSprint(alignment > 0.7)
        } else {
          if (alignment > 0.85) Keybinds.holdSprint(true) else Keybinds.holdSprint(false)
          if (dy > 0.3 && player.isOnGround) {
            val feetPos = player.blockPos
            val groundPos = feetPos.down()
            var needsJump = (isFullBlock(groundPos) && dy > 0.8) || dy > 1.2
            val checkAhead = Vec3d(player.x, player.y, player.z).add(lookDir.x * 0.7, 0.0, lookDir.z * 0.7)
            val headAhead = BlockPos(floor(checkAhead.x).toInt(), floor(player.y + 1).toInt(), floor(checkAhead.z).toInt())
            if (!mc.world!!.getBlockState(headAhead).isAir && player.isOnGround) {
              needsJump = true
            }
            Keybinds.holdJump(needsJump)
          } else {
            Keybinds.holdJump(false)
          }
        }
      } else {
        Keybinds.releaseAll()
      }
    } else {
      Keybinds.releaseAll()
    }
  }

  private fun isWallAhead(player: ClientPlayerEntity, direction: Vec3d): Boolean {
    if (mc.world == null) return false
    val playerPos = Vec3d(player.x, player.y, player.z)
    val checkPos = playerPos.add(direction.x * 0.5, 0.0, direction.z * 0.5)
    val headCheck = BlockPos(floor(checkPos.x).toInt(), floor(playerPos.y + 1).toInt(), floor(checkPos.z).toInt())
    val bodyCheck = BlockPos(floor(checkPos.x).toInt(), floor(playerPos.y).toInt(), floor(checkPos.z).toInt())
    val headState = mc.world!!.getBlockState(headCheck)
    val bodyState = mc.world!!.getBlockState(bodyCheck)
    val bodyBlockName = bodyState.block.translationKey
    val isBodyPartialBlock = (bodyBlockName.contains("slab") || bodyBlockName.contains("stair") || bodyBlockName.contains("carpet"))
    return ((!headState.isAir && headState.blocksMovement()) || (!bodyState.isAir && bodyState.blocksMovement() && !isBodyPartialBlock))
  }

  private fun isFullBlock(pos: BlockPos): Boolean {
    if (mc.world == null) return false
    val state = mc.world!!.getBlockState(pos)
    if (state.isAir) return false
    val blockName = state.block.translationKey
    if (listOf("slab","stair","carpet","pressure_plate","fence","wall","snow","daylight","trap","door","gate","path").any { blockName.contains(it) }) return false
    return state.isFullCube(mc.world, pos)
  }

  fun isMoving(): Boolean = running.get()
  fun getCurrentPath(): List<BlockPos> = currentPath
  fun getCurrentLookaheadPoint(): Vec3d? = currentLookaheadPoint

  // Simple keybind helper using Minecraft options keys
  private object Keybinds {
    private val client get() = MinecraftClient.getInstance()
    fun holdForward(v: Boolean) { client.options.forwardKey.setPressed(v) }
    fun holdJump(v: Boolean) { client.options.jumpKey.setPressed(v) }
    fun holdSprint(v: Boolean) { client.options.sprintKey.setPressed(v) }
    fun releaseAll() { holdForward(false); holdJump(false); holdSprint(false) }
  }

  private class QuarticRotator {
    private var startYaw = 0f
    private var startPitch = 0f
    private var targetYaw = 0f
    private var targetPitch = 0f
    private var startTime = 0L
    private val duration = 300L // Faster updates for tracking

    fun update(player: ClientPlayerEntity, target: Vec3d) {
      val targetRot = AngleUtils.getRotation(target) ?: return

      // Check if we need to reset start (e.g. if we finished previous curve)
      val now = System.currentTimeMillis()
      if (now - startTime > duration) {
        startTime = now
        startYaw = player.yaw
        startPitch = player.pitch
      }

      // Update target constantly
      targetYaw = targetRot.yaw
      targetPitch = targetRot.pitch

      val t = (now - startTime).toFloat() / duration
      val clampedT = t.coerceIn(0f, 1f)
      val ease = 1 - (1 - clampedT).pow(4) // Quartic Ease Out

      val yawDiff = AngleUtils.normalizeYaw(targetYaw - startYaw)
      val newYaw = startYaw + yawDiff * ease
      val newPitch = startPitch + (targetPitch - startPitch) * ease

      player.yaw = newYaw
      player.pitch = newPitch
    }
  }
}
