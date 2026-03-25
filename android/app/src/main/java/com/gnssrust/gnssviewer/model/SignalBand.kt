package com.gnssrust.gnssviewer.model

data class SignalBand(val name: String, val frequencyMhz: Double)

fun classifyBand(constellation: ConstellationType, freqHz: Double): SignalBand {
    val freqMhz = freqHz / 1_000_000.0
    return when (constellation) {
        ConstellationType.GPS -> when {
            freqMhz in 1574.0..1577.0 -> SignalBand("L1", freqMhz)
            freqMhz in 1226.0..1229.0 -> SignalBand("L2", freqMhz)
            freqMhz in 1175.0..1178.0 -> SignalBand("L5", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.GLONASS -> when {
            freqMhz in 1598.0..1610.0 -> SignalBand("G1", freqMhz)
            freqMhz in 1242.0..1252.0 -> SignalBand("G2", freqMhz)
            freqMhz in 1200.0..1205.0 -> SignalBand("G3", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.GALILEO -> when {
            freqMhz in 1574.0..1577.0 -> SignalBand("E1", freqMhz)
            freqMhz in 1175.0..1178.0 -> SignalBand("E5a", freqMhz)
            freqMhz in 1206.0..1209.0 -> SignalBand("E5b", freqMhz)
            freqMhz in 1277.0..1280.0 -> SignalBand("E6", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.BEIDOU -> when {
            freqMhz in 1574.0..1577.0 -> SignalBand("B1C", freqMhz)
            freqMhz in 1560.0..1563.0 -> SignalBand("B1I", freqMhz)
            freqMhz in 1175.0..1178.0 -> SignalBand("B2a", freqMhz)
            freqMhz in 1206.0..1209.0 -> SignalBand("B2b", freqMhz)
            freqMhz in 1267.0..1270.0 -> SignalBand("B3I", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.QZSS -> when {
            freqMhz in 1574.0..1577.0 -> SignalBand("L1", freqMhz)
            freqMhz in 1226.0..1229.0 -> SignalBand("L2", freqMhz)
            freqMhz in 1175.0..1178.0 -> SignalBand("L5", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.SBAS -> when {
            freqMhz in 1574.0..1577.0 -> SignalBand("L1", freqMhz)
            freqMhz in 1175.0..1178.0 -> SignalBand("L5", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.IRNSS -> when {
            freqMhz in 1174.0..1178.0 -> SignalBand("L5", freqMhz)
            freqMhz in 1191.0..1196.0 -> SignalBand("S", freqMhz)
            else -> SignalBand("?", freqMhz)
        }
        ConstellationType.UNKNOWN -> SignalBand("?", freqMhz)
    }
}
