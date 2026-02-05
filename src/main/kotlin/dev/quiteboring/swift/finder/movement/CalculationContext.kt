package dev.quiteboring.swift.finder.movement

import dev.quiteboring.swift.finder.costs.ActionCosts
import dev.quiteboring.swift.util.BlockStateAccessor
import net.minecraft.block.BlockState
import net.minecraft.client.MinecraftClient
import net.minecraft.entity.effect.StatusEffects

class CalculationContext {

  val mc: MinecraftClient = MinecraftClient.getInstance()
  val world = mc.world!!
  val player = mc.player

  val bsa = BlockStateAccessor(world)
  val jumpBoostAmplifier = player?.getStatusEffect(StatusEffects.JUMP_BOOST)?.amplifier ?: -1
  val cost = ActionCosts(jumpBoostAmplifier = jumpBoostAmplifier)
  val maxFallHeight = 20

  val wdc = WallDistanceCalculator(this)

  fun get(x: Int, y: Int, z: Int): BlockState? {
    return bsa.get(x, y, z)
  }

}
