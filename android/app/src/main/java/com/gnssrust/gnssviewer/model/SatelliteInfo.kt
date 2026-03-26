package com.gnssrust.gnssviewer.model

data class SatelliteInfo(
    val constellation: ConstellationType,
    val svid: Int,
    val cn0DbHz: Float,
    val elevationDeg: Float,
    val azimuthDeg: Float,
    val hasEphemeris: Boolean,
    val hasAlmanac: Boolean,
    val usedInFix: Boolean,
    val carrierFreqHz: Double? = null,
    val band: SignalBand? = null,
    val basebandCn0DbHz: Float? = null,
    val dopplerMps: Double? = null,
    val agcDb: Double? = null,
    val trackingState: TrackingState? = null,
    val multipathIndicator: Int? = null,
    val statusIndex: Int = 0
) {
    val key: String
        get() = "${statusIndex}_${constellation.shortName}${svid}_${band?.name ?: "x"}"

    val multipathString: String
        get() = when (multipathIndicator) {
            0 -> "Unknown"
            1 -> "Detected"
            2 -> "Not detected"
            else -> "-"
        }
}
