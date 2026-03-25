package com.gnssrust.gnssviewer.model

data class TrackingState(val flags: Int) {
    val hasCodeLock: Boolean get() = flags and (1 shl 0) != 0
    val hasBitSync: Boolean get() = flags and (1 shl 1) != 0
    val hasSubframeSync: Boolean get() = flags and (1 shl 2) != 0
    val hasTowDecoded: Boolean get() = flags and (1 shl 3) != 0
    val hasMsecAmbiguity: Boolean get() = flags and (1 shl 4) != 0

    fun toShortString(): String {
        val parts = mutableListOf<String>()
        if (hasCodeLock) parts.add("CODE")
        if (hasBitSync) parts.add("BIT")
        if (hasSubframeSync) parts.add("SF")
        if (hasTowDecoded) parts.add("TOW")
        if (hasMsecAmbiguity) parts.add("MS")
        return parts.joinToString("|").ifEmpty { "-" }
    }
}
