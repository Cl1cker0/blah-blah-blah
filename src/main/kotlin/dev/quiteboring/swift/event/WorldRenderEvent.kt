package dev.quiteboring.swift.event

import net.fabricmc.fabric.api.event.Event
import net.fabricmc.fabric.api.event.EventFactory
import net.minecraft.client.render.Camera
import net.minecraft.client.render.Frustum
import net.minecraft.client.render.VertexConsumerProvider
import net.minecraft.client.util.math.MatrixStack

object WorldRenderEvent {

  fun interface StartEvent {
    fun trigger(ctx: Context)
  }

  fun interface LastEvent {
    fun trigger(ctx: Context)
  }

  @JvmField
  val START = bake<StartEvent> { v -> StartEvent { ctx -> v.forEach { it.trigger(ctx) } } }

  @JvmField
  val LAST = bake<LastEvent> { v -> LastEvent { ctx -> v.forEach { it.trigger(ctx) } } }

  private inline fun <reified T> bake(noinline v: (Array<T>) -> T): Event<T> =
    EventFactory.createArrayBacked(T::class.java, v)
}

class Context {
  var matrixStack: MatrixStack? = null
  lateinit var consumers: VertexConsumerProvider
  lateinit var camera: Camera
  lateinit var frustum: Frustum
}
