package dev.quiteboring.swift.finder.movement

import dev.quiteboring.swift.finder.movement.movements.*

enum class Moves(val offsetX: Int, val offsetZ: Int) {

  TRAVERSE_NORTH(0, -1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementTraverse.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  TRAVERSE_SOUTH(0, +1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementTraverse.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  TRAVERSE_EAST(+1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementTraverse.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  TRAVERSE_WEST(-1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementTraverse.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  ASCEND_NORTH(0, -1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementAscend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  ASCEND_SOUTH(0, +1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementAscend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  ASCEND_EAST(+1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementAscend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  ASCEND_WEST(-1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementAscend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DESCEND_NORTH(0, -1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDescend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DESCEND_SOUTH(0, +1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDescend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DESCEND_EAST(+1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDescend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DESCEND_WEST(-1, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDescend.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DIAGONAL_NORTHEAST(+1, -1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDiagonal.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DIAGONAL_NORTHWEST(-1, -1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDiagonal.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DIAGONAL_SOUTHEAST(+1, +1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDiagonal.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  DIAGONAL_SOUTHWEST(-1, +1) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementDiagonal.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_1_NORTH(0, -2) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_1_SOUTH(0, +2) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_1_EAST(+2, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_1_WEST(-2, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_2_NORTH(0, -3) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_2_SOUTH(0, +3) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_2_EAST(+3, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  },

  JUMP_GAP_2_WEST(-3, 0) {
    override fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult) {
      MovementJumpGap.calculateCost(ctx, parentX, parentY, parentZ, parentX + offsetX, parentZ + offsetZ, res)
    }
  };

  abstract fun calculate(ctx: CalculationContext, parentX: Int, parentY: Int, parentZ: Int, res: MovementResult)

}
