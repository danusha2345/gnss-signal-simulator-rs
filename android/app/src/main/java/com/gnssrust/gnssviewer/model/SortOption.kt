package com.gnssrust.gnssviewer.model

enum class SortOption(val label: String) {
    BY_SYSTEM("System"),
    BY_SVID("SVID"),
    BY_CN0("CN0 ↓"),
    BY_ELEVATION("Elev ↓"),
    BY_AZIMUTH("Az"),
    BY_USED_IN_FIX("In Fix"),
    BY_BAND("Band");
}
