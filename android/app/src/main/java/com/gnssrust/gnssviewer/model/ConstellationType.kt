package com.gnssrust.gnssviewer.model

enum class ConstellationType(
    val displayName: String,
    val shortName: String,
    val colorArgb: Long,
    val sortOrder: Int
) {
    GPS("GPS", "G", 0xFF4285F4, 0),
    GLONASS("GLONASS", "R", 0xFFEA4335, 1),
    GALILEO("Galileo", "E", 0xFFFFA726, 2),
    BEIDOU("BeiDou", "C", 0xFF34A853, 3),
    QZSS("QZSS", "J", 0xFF9C27B0, 4),
    SBAS("SBAS", "S", 0xFF9E9E9E, 5),
    IRNSS("IRNSS", "I", 0xFF00BCD4, 6),
    UNKNOWN("Unknown", "?", 0xFF757575, 7);

    companion object {
        fun fromAndroidType(type: Int): ConstellationType = when (type) {
            1 -> GPS
            2 -> SBAS
            3 -> GLONASS
            4 -> QZSS
            5 -> BEIDOU
            6 -> GALILEO
            7 -> IRNSS
            else -> UNKNOWN
        }
    }
}
