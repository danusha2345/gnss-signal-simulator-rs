package com.gnssrust.gnssviewer.model

data class FixInfo(
    val hasFix: Boolean = false,
    val latitude: Double = 0.0,
    val longitude: Double = 0.0,
    val altitude: Double = 0.0,
    val accuracy: Float = 0f,
    val verticalAccuracy: Float? = null,
    val speedAccuracy: Float? = null,
    val speed: Float = 0f,
    val bearing: Float = 0f,
    val satellitesUsed: Int = 0,
    val timestamp: Long = 0L
)
