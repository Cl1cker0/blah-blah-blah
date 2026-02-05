package dev.quiteboring.swift.util

import net.minecraft.client.MinecraftClient
import net.minecraft.util.math.Vec3d

/**
 * Kotlin port of the provided AngleUtils Java class.
 * Implemented as an object so calls like AngleUtils.normalizeYaw(...) work.
 */
object AngleUtils {
  private val mc: MinecraftClient = MinecraftClient.getInstance()

  fun getPlayerRotation(): Rotations.Rotation? {
    val player = mc.player ?: return null
    return Rotations.Rotation(player.yaw, player.pitch)
  }

  fun wrapAngleTo180(value: Double): Double {
    val it = value % 360.0
    return when {
      it >= 180.0 -> it - 360.0
      it <= -180.0 -> it + 360.0
      else -> it
    }
  }

  fun wrapAngleTo180(angle: Float): Float = wrapAngleTo180(angle.toDouble()).toFloat()

  fun normalizeAngle(angle: Float): Float {
    var a = angle
    while (a > 180.0f) a -= 360.0f
    while (a <= -180.0f) a += 360.0f
    return a
  }

  fun normalizeYaw(yaw: Float): Float {
    var newYaw = yaw % 360.0f
    if (newYaw < -180.0f) newYaw += 360.0f
    if (newYaw > 180.0f) newYaw -= 360.0f
    return newYaw
  }

  fun getRotation(to: Vec3d): Rotations.Rotation? {
    val player = mc.player ?: return null
    val eyePos = player.eyePos
    return getRotation(eyePos, to)
  }

  fun getRotation(from: Vec3d, to: Vec3d): Rotations.Rotation {
    val xDiff = to.x - from.x
    val yDiff = to.y - from.y
    val zDiff = to.z - from.z
    val dist = kotlin.math.sqrt(xDiff * xDiff + zDiff * zDiff)

    val yaw = (kotlin.math.atan2(zDiff, xDiff) * (180.0 / Math.PI)).toFloat() - 90f
    val pitch = -(kotlin.math.atan2(yDiff, dist) * (180.0 / Math.PI)).toFloat()

    return Rotations.Rotation(yaw, pitch)
  }

}
