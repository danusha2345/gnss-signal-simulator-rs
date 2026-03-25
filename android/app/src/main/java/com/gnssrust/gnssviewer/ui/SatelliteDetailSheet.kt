package com.gnssrust.gnssviewer.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gnssrust.gnssviewer.model.SatelliteInfo
import com.gnssrust.gnssviewer.ui.theme.*

@Composable
fun SatelliteDetailSheet(
    sat: SatelliteInfo,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        confirmButton = {
            TextButton(onClick = onDismiss) { Text("Close") }
        },
        title = {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Text(
                    text = sat.constellation.shortName,
                    color = Color(sat.constellation.colorArgb),
                    fontWeight = FontWeight.Bold,
                    fontSize = 24.sp
                )
                Spacer(Modifier.width(8.dp))
                Text(
                    text = "${sat.constellation.displayName} ${sat.svid}",
                    fontSize = 20.sp,
                    fontWeight = FontWeight.Bold
                )
                if (sat.usedInFix) {
                    Spacer(Modifier.width(8.dp))
                    Text("★", fontSize = 20.sp, color = Color.Yellow)
                }
            }
        },
        text = {
            Column(
                modifier = Modifier
                    .fillMaxWidth()
                    .verticalScroll(rememberScrollState())
            ) {
                // Signal section
                SectionHeader("Signal")
                DetailRow("CN0", "%.1f dB-Hz".format(sat.cn0DbHz))
                Cn0Bar(sat.cn0DbHz, modifier = Modifier.fillMaxWidth().padding(vertical = 4.dp))
                sat.basebandCn0DbHz?.let {
                    DetailRow("Baseband CN0", "%.1f dB-Hz".format(it))
                }
                sat.agcDb?.let {
                    DetailRow("AGC Level", "%.1f dB".format(it))
                }

                Spacer(Modifier.height(12.dp))

                // Position section
                SectionHeader("Position")
                DetailRow("Elevation", "%.1f°".format(sat.elevationDeg))
                DetailRow("Azimuth", "%.1f°".format(sat.azimuthDeg))

                Spacer(Modifier.height(12.dp))

                // Frequency section
                SectionHeader("Frequency")
                DetailRow("Band", sat.band?.name ?: "Unknown")
                sat.carrierFreqHz?.let {
                    DetailRow("Carrier Freq", "%.3f MHz".format(it / 1_000_000.0))
                }

                Spacer(Modifier.height(12.dp))

                // Doppler section
                sat.dopplerMps?.let { doppler ->
                    SectionHeader("Doppler")
                    DetailRow("Pseudorange Rate", "%.2f m/s".format(doppler))
                    DetailRow("Doppler Shift", "%.1f Hz".format(
                        doppler * (sat.carrierFreqHz ?: 1575420000.0) / 299792458.0
                    ))
                    Spacer(Modifier.height(12.dp))
                }

                // Status section
                SectionHeader("Status")
                DetailRow("Used in Fix", if (sat.usedInFix) "Yes" else "No")
                DetailRow("Has Ephemeris", if (sat.hasEphemeris) "Yes" else "No")
                DetailRow("Has Almanac", if (sat.hasAlmanac) "Yes" else "No")

                sat.trackingState?.let { ts ->
                    Spacer(Modifier.height(12.dp))
                    SectionHeader("Tracking State")
                    DetailRow("Code Lock", flagText(ts.hasCodeLock))
                    DetailRow("Bit Sync", flagText(ts.hasBitSync))
                    DetailRow("Subframe Sync", flagText(ts.hasSubframeSync))
                    DetailRow("TOW Decoded", flagText(ts.hasTowDecoded))
                    DetailRow("Msec Ambiguity", flagText(ts.hasMsecAmbiguity))
                }

                sat.multipathIndicator?.let {
                    Spacer(Modifier.height(12.dp))
                    SectionHeader("Multipath")
                    DetailRow("Indicator", sat.multipathString)
                }
            }
        }
    )
}

@Composable
private fun SectionHeader(title: String) {
    Text(
        text = title,
        fontSize = 14.sp,
        fontWeight = FontWeight.Bold,
        color = Color(0xFF90CAF9),
        modifier = Modifier.padding(bottom = 4.dp)
    )
}

@Composable
private fun DetailRow(label: String, value: String) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 2.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = label,
            fontSize = 13.sp,
            color = Color.Gray
        )
        Text(
            text = value,
            fontSize = 13.sp,
            fontFamily = FontFamily.Monospace,
            color = Color.White
        )
    }
}

private fun flagText(flag: Boolean): String = if (flag) "Yes" else "No"
