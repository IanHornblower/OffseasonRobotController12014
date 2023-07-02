package org.firstinspires.ftc.teamcode.util.motionprofile

/**
 * Motion profile velocity constraint.
 */
fun interface VelocityConstraint {

    /**
     * Returns the maximum profile velocity at displacement [s].
     */
    operator fun get(s: Double): Double
}