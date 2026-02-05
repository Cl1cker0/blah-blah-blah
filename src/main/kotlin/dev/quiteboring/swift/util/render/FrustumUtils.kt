package dev.quiteboring.swift.util.render

import dev.quiteboring.swift.mixins.FrustumInvoker
import net.minecraft.client.render.Frustum
import net.minecraft.util.math.Box
import org.joml.FrustumIntersection

object FrustumUtils {

  fun isVisible(frustum: Frustum, box: Box): Boolean {
    return isVisible(frustum, box)
  }

  fun isVisible(
    frustum: Frustum,
    minX: Double,
    minY: Double,
    minZ: Double,
    maxX: Double,
    maxY: Double,
    maxZ: Double,
  ): Boolean {
    val plane: Int = (frustum as FrustumInvoker).invokeIntersectAab(minX, minY, minZ, maxX, maxY, maxZ)

    return plane == FrustumIntersection.INSIDE || plane == FrustumIntersection.INTERSECT
  }

}
