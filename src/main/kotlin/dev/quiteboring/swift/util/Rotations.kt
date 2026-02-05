package dev.quiteboring.swift.util

import kotlin.math.*
import dev.quiteboring.swift.event.WorldRenderEvent
import dev.quiteboring.swift.event.Context
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents
import net.minecraft.client.MinecraftClient
import net.minecraft.util.math.MathHelper
import net.minecraft.util.math.Vec3d
import dev.quiteboring.swift.api.PathManager

/**
 * Rotation utilities with gentle render-based interpolation suitable for
 * Fabric 1.21.10 usage inside this mod's event system.
 */
class Rotations {
  /**
   * Inner class to hold rotation data (yaw and pitch)
   */
  class Rotation(val yaw: Float, val pitch: Float) {
    fun component1(): Float {
      return this.yaw
    }

    fun component2(): Float {
      return this.pitch
    }

    fun copy(yaw: Float, pitch: Float): Rotation {
      return Rotation(yaw, pitch)
    }

    override fun toString(): String {
      return "Rotation(yaw=" + this.yaw + ", pitch=" + this.pitch + ")"
    }

    override fun hashCode(): Int {
      var result = java.lang.Float.hashCode(this.yaw)
      result = result * 31 + java.lang.Float.hashCode(this.pitch)
      return result
    }

    override fun equals(other: Any?): Boolean {
      if (this === other) {
        return true
      }
      if (other !is Rotation) {
        return false
      }
      val rotation = other
      return java.lang.Float.compare(this.yaw, rotation.yaw) == 0 && java.lang.Float.compare(
        this.pitch,
        rotation.pitch
      ) == 0
    }
  }

  // Interpolation for smooth rendering
  private var targetYaw = 0f
  private var targetPitch = 0f
  private var directAngleMode = false // Flag for direct angle rotation
  private var currentYaw = 0f
  private var currentPitch = 0f

  // Render-based smooth rotation
  private var useRenderInterpolation = false
  private var lastRenderTime = 0L

  // Track pathfinding state to only ungrab mouse on transition
  private var wasPathfindingActive = false

  // Rotation modes
  enum class Mode {
    NONE,
    ROTATING,
    TRACKING
  }

  private var mode = Mode.NONE

  /**
   * Check if currently rotating
   */
  var isRotating: Boolean = false
    private set
  private var yawOnly = false
  private var reachedEnd = false
  private var targetVec: Vec3d? = null
  private var trackingTarget: Vec3d? = null // For tracking mode
  private var startYaw = 0f
  private var startPitch = 0f
  private var precision = 1.0f
  private var rotationStartTime = 0L
  private val endActions: MutableList<Runnable> = ArrayList<Runnable>()

  // Velocity-based rotation variables
  private var yawVelocity = 0.0f
  private var pitchVelocity = 0.0f

  /**
   * Initialize render-based interpolation for smooth rotations
   */
  private fun initRenderInterpolation() {
    // Use the mod's render event so we don't depend on Fabric's direct render event mapping
    WorldRenderEvent.LAST.register { ctx: Context ->
      if (MinecraftClient.getInstance().player == null || !useRenderInterpolation) return@register
      val currentTime = System.currentTimeMillis()
      val deltaTime = min((currentTime - lastRenderTime) / 16.67f, 2.0f) // Cap at 2x for stability
      lastRenderTime = currentTime

      // Smooth interpolation towards target
      if (mode != Mode.NONE && this.isRotating) {
        interpolateRotation(deltaTime)
      }
    }

    // Add rotation control with conditional cursor ungrabbing on state transition
    ClientTickEvents.END_CLIENT_TICK.register(ClientTickEvents.EndTick { client: MinecraftClient? ->
      if (client == null || client.player == null) return@EndTick

      // Check if pathfinding is active via PathManager (safe fallback)
      val pathfindingActive: Boolean = PathManager.getPathSize() > 0

      // Only ungrab mouse when transitioning from active to inactive (not continuously)
      if (wasPathfindingActive && !pathfindingActive) {
        // Pathfinding just stopped - ungrab the mouse cursor for Alt+Tab
        try {
          val mouse = client.mouse
          if (mouse != null) {
            // In newer mappings the method is 'unlockCursor' on the mouse
            mouse.unlockCursor()
          }
        } catch (ignored: Throwable) {
        }
      }

      // Update state tracking
      wasPathfindingActive = pathfindingActive

      // Only apply rotation corrections during pathfinding and active rotation mode
      if (!pathfindingActive || mode == Mode.NONE || !this.isRotating) return@EndTick

      // Apply gentle rotation correction for pathfinding
      val playerYaw = client.player!!.yaw
      val playerPitch = client.player!!.pitch

      // Calculate rotation difference
      val yawDiff: Float = AngleUtils.normalizeYaw(targetYaw - playerYaw)
      val pitchDiff = targetPitch - playerPitch

      // Apply gentle interpolation towards target (reduced speed for natural feel)
      val tickSpeed = 0.15f // Gentle correction speed
      currentYaw = playerYaw + (yawDiff * tickSpeed)
      currentPitch = playerPitch + (pitchDiff * tickSpeed)

      // Clamp pitch
      currentPitch = MathHelper.clamp(currentPitch, -90f, 90f)

      // Apply rotation correction
      try {
        client.player!!.yaw = currentYaw
        client.player!!.pitch = currentPitch
      } catch (ignored: Throwable) {
      }
    })
  }

  /**
   * Smooth interpolation for render-based rotations with gentle cursor freedom
   */
  private fun interpolateRotation(deltaTime: Float) {
    val player = MinecraftClient.getInstance().player ?: return

    // Get current player rotation
    val playerYaw = player.yaw
    val playerPitch = player.pitch

    // Calculate difference to target
    val yawDiff: Float = AngleUtils.normalizeYaw(targetYaw - playerYaw)
    val pitchDiff = targetPitch - playerPitch

    // Gentle interpolation - reduced for natural feel with cursor freedom
    val interpolationSpeed = min(RENDER_SMOOTHING * deltaTime * 1.5f, 0.3f) // Much gentler
    val newYaw = playerYaw + (yawDiff * interpolationSpeed)
    var newPitch = playerPitch + (pitchDiff * interpolationSpeed)

    // Clamp pitch
    newPitch = MathHelper.clamp(newPitch, -90f, 90f)

    // Apply gentle rotation correction (single application)
    try {
      player.yaw = newYaw
      player.pitch = newPitch
    } catch (ignored: Throwable) {
    }

    // Update our tracking variables
    currentYaw = newYaw
    currentPitch = newPitch

    // Check if we've reached the target
    if (abs(yawDiff) < 1f && abs(pitchDiff) < 1f) {
      if (mode == Mode.ROTATING) {
        triggerEndRotation()
      }
    }
  }

  /**
   * Start rotating to a target vector position - now with render interpolation
   */
  fun rotateTo(target: Vec3d, precision: Float, yawOnly: Boolean, pitch: Float) {
    this.mode = Mode.ROTATING
    this.isRotating = true
    this.rotationStartTime = System.currentTimeMillis()
    this.lastRenderTime = System.currentTimeMillis()

    // Don't restart if already rotating to the same target
    if (!this.reachedEnd && this.targetVec != null && this.isRotating && target.x == this.targetVec!!.x && target.y == this.targetVec!!.y && target.z == this.targetVec!!.z) {
      return
    }

    this.targetVec = target
    this.trackingTarget = null // Clear tracking
    this.yawOnly = yawOnly
    this.targetPitch = pitch
    val player = MinecraftClient.getInstance().player
    if (player != null) {
      this.startYaw = player.yaw
      this.startPitch = player.pitch
    }
    this.precision = precision
    this.reachedEnd = false

    // Initialize render interpolation
    this.currentYaw = this.startYaw
    this.currentPitch = this.startPitch
    val targetRot = getAngles(target)
    if (targetRot != null) {
      this.targetYaw = targetRot.yaw
      this.targetPitch = if (yawOnly) pitch else targetRot.pitch
    }
    this.useRenderInterpolation = true
  }

  /**
   * Start tracking a target with smooth render-based rotation
   */
  fun trackTarget(target: Vec3d, trackingSpeed: Float) {
    this.mode = Mode.TRACKING
    this.trackingTarget = target
    this.yawVelocity = 0.0f
    this.pitchVelocity = 0.0f
    this.isRotating = true
    this.rotationStartTime = System.currentTimeMillis()
    this.lastRenderTime = System.currentTimeMillis()

    // Initialize render interpolation for tracking
    val player = MinecraftClient.getInstance().player
    if (player != null) {
      this.currentYaw = player.yaw
      this.currentPitch = player.pitch

      val targetRot = getAngles(target)
      if (targetRot != null) {
        this.targetYaw = targetRot.yaw
        this.targetPitch = targetRot.pitch
      }
    }
    this.useRenderInterpolation = true
  }

  /**
   * Rotate to specific yaw and pitch angles with render interpolation
   */
  fun rotateToAngles(yaw: Float, pitch: Float, precision: Float, yawOnly: Boolean) {
    this.mode = Mode.ROTATING
    this.isRotating = true
    this.rotationStartTime = System.currentTimeMillis()
    this.lastRenderTime = System.currentTimeMillis()
    this.targetVec = null // No need for fake vector in direct angle mode
    this.trackingTarget = null // Clear tracking
    val player = MinecraftClient.getInstance().player
    if (player != null) {
      this.startYaw = player.yaw
      this.startPitch = player.pitch
    }
    this.precision = precision
    this.yawOnly = yawOnly
    this.reachedEnd = false
    this.directAngleMode = true // Enable direct angle mode

    // Initialize render interpolation for direct angles
    this.currentYaw = this.startYaw
    this.currentPitch = this.startPitch
    this.targetYaw = yaw
    this.targetPitch = pitch // Always use the provided pitch directly
    this.useRenderInterpolation = true
  }

  /**
   * Stop rotating and disable render interpolation
   */
  fun stopRotate() {
    this.mode = Mode.NONE
    this.isRotating = false
    this.targetVec = null
    this.trackingTarget = null
    this.yawOnly = false
    this.directAngleMode = false
    this.endActions.clear()
    this.yawVelocity = 0.0f
    this.pitchVelocity = 0.0f
    this.useRenderInterpolation = false
  }

  /**
   * Add callback for when rotation ends
   */
  fun onEndRotation(callback: Runnable?) {
    callback?.let { this.endActions.add(it) }
  }

  /**
   * Update rotation - call this every tick (now works with render interpolation)
   */
  fun update() {
    val player = MinecraftClient.getInstance().player
    if (mode == Mode.NONE || player == null) {
      return
    }

    // Check for timeout to prevent getting stuck
    if (System.currentTimeMillis() - rotationStartTime > ROTATION_TIMEOUT_MS) {
      stopRotate() // Force stop rotation after timeout
      return
    }

    // Update target rotation for tracking mode
    if (mode == Mode.TRACKING && trackingTarget != null) {
      val newTarget = getAngles(trackingTarget!!)
      if (newTarget != null) {
        this.targetYaw = newTarget.yaw
        this.targetPitch = newTarget.pitch
      }
    }

    // The actual rotation interpolation is handled by the render event
    // This method now mainly updates targets and handles timeouts
  }

  private fun triggerEndRotation() {
    this.mode = Mode.NONE
    this.isRotating = false
    this.useRenderInterpolation = false

    // Apply final rotation
    if (this.targetVec != null) {
      val finalRot = getAngles(this.targetVec!!)
      if (finalRot != null) {
        val player = MinecraftClient.getInstance().player
        if (player != null) {
          player.yaw = finalRot.yaw
          player.pitch = if (this.yawOnly) this.targetPitch else finalRot.pitch
        }
      }
    }

    // Execute end actions
    for (action in this.endActions) {
      action.run()
    }
    this.endActions.clear()
    this.yawOnly = false
    this.reachedEnd = true
  }

  /**
   * Calculate the needed rotation change between two rotations
   */
  private fun getNeededChange(startRot: Rotation, endRot: Rotation): Rotation {
    // Use proper angle wrapping to prevent 180-degree flips
    val yawDiff: Float = AngleUtils.normalizeYaw(endRot.yaw - startRot.yaw)
    var pitchDiff = endRot.pitch - startRot.pitch

    // Clamp pitch difference to prevent extreme movements
    pitchDiff = MathHelper.clamp(pitchDiff, -45.0f, 45.0f)

    return Rotation(yawDiff, pitchDiff)
  }

  private fun getAngles(target: Vec3d): Rotation {
    val player = MinecraftClient.getInstance().player ?: return Rotation(0f, 0f)
    val playerPos: Vec3d = player.eyePos

    val deltaX = target.x - playerPos.x
    val deltaY = target.y - (playerPos.y + player.getEyeHeight(player.pose))
    val deltaZ = target.z - playerPos.z

    // Check if target is too close (less than 1 block away horizontally)
    val horizontalDistance = sqrt(deltaX * deltaX + deltaZ * deltaZ)
    if (horizontalDistance < 1.0) {
      // For very close targets, just look straight ahead with slight downward pitch
      val currentYaw = player.yaw
      return Rotation(currentYaw, 10f) // Look slightly down
    }

    val yaw = (atan2(deltaZ, deltaX) * (180 / Math.PI)).toFloat() - 90f
    var pitch = -(atan2(deltaY, horizontalDistance) * (180 / Math.PI)).toFloat()

    // Clamp pitch to reasonable values to prevent looking straight up/down
    pitch = MathHelper.clamp(pitch, -60f, 45f)

    return Rotation(yaw, pitch)
  }

  private fun interpolate(targetRotation: Rotation): Rotation {
    val player = MinecraftClient.getInstance().player ?: return targetRotation
    val lastRotation = Rotation(
      player.yaw,
      player.pitch
    )

    // Get diffs and distance - use proper angle wrapping
    val deltaYaw: Float = AngleUtils.normalizeYaw(targetRotation.yaw - lastRotation.yaw)
    var deltaPitch = targetRotation.pitch - lastRotation.pitch

    // Clamp pitch to prevent extreme movements
    deltaPitch = MathHelper.clamp(deltaPitch, -30.0f, 30.0f)

    val distance = sqrt((deltaYaw * deltaYaw + deltaPitch * deltaPitch).toDouble()).toFloat()

    if (distance == 0f) return targetRotation

    // Apply rotation speed and distance modifiers
    val diffYaw: Float = Math.abs(AngleUtils.normalizeYaw(this.startYaw - targetRotation.yaw))
    val diffPitch = abs(targetRotation.pitch - this.startPitch)

    val maxYaw = (ROTATION_SPEED / 2f) * abs(deltaYaw / distance) * fader(diffYaw)
    val maxPitch = (ROTATION_SPEED / 2f) * abs(deltaPitch / distance) * fader(diffPitch)

    // Clamp movement
    val moveYaw = MathHelper.clamp(deltaYaw, -maxYaw, maxYaw) + randomness() * fader(deltaYaw)
    val movePitch = MathHelper.clamp(deltaPitch, -maxPitch, maxPitch) + randomness() * fader(deltaPitch)

    var newYaw = lastRotation.yaw + moveYaw
    var newPitch = lastRotation.pitch + movePitch

    // Add minimal tremors only if sensitivity is high enough
    val sensitivity = this.sensitivity
    if (sensitivity > 0.8f) {
      val tremorStrength = abs(sensitivity) * 0.1f // Reduced tremor strength
      newYaw += (tremorStrength * sin((TREMOR_FREQUENCY * System.currentTimeMillis() / 1000f).toDouble())).toFloat()
      newPitch += (tremorStrength * cos((TREMOR_FREQUENCY * System.currentTimeMillis() / 1000f).toDouble())).toFloat()
    }

    // Apply sensitivity multiple times for smoothness
    for (i in 0..2) {
      val adjusted = applySensitivity(Rotation(newYaw, newPitch))
      newYaw = adjusted.yaw
      newPitch = MathHelper.clamp(adjusted.pitch, -90f, 90f)
    }

    return Rotation(newYaw, newPitch)
  }

  private fun applySensitivity(rotation: Rotation): Rotation {
    val player = MinecraftClient.getInstance().player ?: return rotation
    val currentRotation = Rotation(
      player.yaw,
      player.pitch
    )

    val multiplier = this.sensitivity.toDouble().pow(3.0).toFloat() * 8f * 0.15f
    val yaw = currentRotation.yaw + Math.round((rotation.yaw - currentRotation.yaw) / multiplier) * multiplier
    val pitch = currentRotation.pitch + Math.round((rotation.pitch - currentRotation.pitch) / multiplier) * multiplier

    return Rotation(yaw, MathHelper.clamp(pitch, -90f, 90f))
  }

  private fun randomness(): Float {
    return ((Math.random() - 0.5) * RANDOMNESS).toFloat()
  }

  private fun fader(diff: Float): Float {
    return (1 - exp((-abs(diff) * FADER_EXPONENT).toDouble())).toFloat()
  }

  private val sensitivity: Float
    get() {
      val options = MinecraftClient.getInstance().options
      if (options == null) return 0.5f
      // In newer mappings options.mouseSensitivity is a SliderOption with getValue()
      return options.getMouseSensitivity().getValue().toFloat() * (1f + Math.random().toFloat() / 10000000f) * 0.6f + 0.2f
    }

  private fun get180Yaw(yaw: Float): Float {
    return MathHelper.wrapDegrees(yaw)
  }

  companion object {
    // Rotation settings - Enhanced for smooth rendering
    private const val ROTATION_SPEED = 5f // Much slower for human-like movement
    private const val TRACKING_SPEED = 0.15f // Slower tracking
    private const val RANDOMNESS = 2.5f // More randomness for human feel
    private const val TREMOR_FREQUENCY = 15f // More tremor
    private const val FADER_EXPONENT = 0.05f // Smoother fading
    private const val ROTATION_TIMEOUT_MS: Long = 5000 // Longer timeout
    private const val MAX_VELOCITY = 4f // Much slower max velocity
    private const val SMOOTHNESS = 0.9f // Even higher smoothing

    private const val RENDER_SMOOTHING = 0.15f // Smooth interpolation factor

    var instance: Rotations? = null
      get() {
        if (field == null) {
          field = Rotations()
          field!!.initRenderInterpolation()
        }
        return field
      }
      private set
  }
}
