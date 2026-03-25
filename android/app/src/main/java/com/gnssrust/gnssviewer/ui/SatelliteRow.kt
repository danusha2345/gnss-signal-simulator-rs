package com.gnssrust.gnssviewer.ui

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.*
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gnssrust.gnssviewer.model.SatelliteInfo
import com.gnssrust.gnssviewer.ui.theme.Cn0Medium
import com.gnssrust.gnssviewer.ui.theme.Cn0Strong

@Composable
fun SatelliteRow(sat: SatelliteInfo, onClick: () -> Unit) {
    val bgColor = if (sat.usedInFix)
        MaterialTheme.colorScheme.surfaceVariant.copy(alpha = 0.7f)
    else
        MaterialTheme.colorScheme.surface

    Surface(
        color = bgColor,
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 4.dp, vertical = 1.dp)
            .clickable(onClick = onClick)
    ) {
        Column(modifier = Modifier.padding(horizontal = 8.dp, vertical = 3.dp)) {
            // Line 1: main data
            Row(verticalAlignment = Alignment.CenterVertically) {
                // System badge
                Text(
                    text = sat.constellation.shortName,
                    color = Color(sat.constellation.colorArgb),
                    fontWeight = FontWeight.Bold,
                    fontFamily = FontFamily.Monospace,
                    fontSize = 14.sp,
                    modifier = Modifier.width(18.dp)
                )
                // SVID
                Text(
                    text = "%3d".format(sat.svid),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 13.sp,
                    color = Color.White,
                    modifier = Modifier.width(30.dp)
                )
                Spacer(Modifier.width(4.dp))
                // Band
                Text(
                    text = (sat.band?.name ?: "?").padEnd(3),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 11.sp,
                    color = Color.LightGray,
                    modifier = Modifier.width(30.dp)
                )
                Spacer(Modifier.width(4.dp))
                // CN0 bar
                Cn0Bar(
                    cn0 = sat.cn0DbHz,
                    modifier = Modifier.width(56.dp)
                )
                Spacer(Modifier.width(4.dp))
                // CN0 value
                Text(
                    text = "%4.1f".format(sat.cn0DbHz),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 12.sp,
                    color = Color.White,
                    modifier = Modifier.width(38.dp)
                )
                // Elevation
                Text(
                    text = "%2.0f°".format(sat.elevationDeg),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 12.sp,
                    color = Color.LightGray,
                    modifier = Modifier.width(32.dp)
                )
                // Azimuth
                Text(
                    text = "%3.0f°".format(sat.azimuthDeg),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 12.sp,
                    color = Color.LightGray,
                    modifier = Modifier.width(36.dp)
                )
                // Doppler
                Text(
                    text = sat.dopplerMps?.let { "%+.0f".format(it) } ?: "",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 11.sp,
                    color = Color.Gray,
                    modifier = Modifier.width(48.dp)
                )
            }

            // Line 2: flags and extra info
            Row(verticalAlignment = Alignment.CenterVertically) {
                Spacer(Modifier.width(52.dp))
                // Tracking state
                Text(
                    text = sat.trackingState?.toShortString() ?: "",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 9.sp,
                    color = Color(0xFF888888),
                    modifier = Modifier.weight(1f)
                )
                // Ephemeris flag
                if (sat.hasEphemeris) {
                    Text("E", fontSize = 10.sp, color = Cn0Strong,
                        fontWeight = FontWeight.Bold)
                    Spacer(Modifier.width(3.dp))
                }
                // Almanac flag
                if (sat.hasAlmanac) {
                    Text("A", fontSize = 10.sp, color = Cn0Medium,
                        fontWeight = FontWeight.Bold)
                    Spacer(Modifier.width(3.dp))
                }
                // Used in fix
                if (sat.usedInFix) {
                    Text("★", fontSize = 12.sp, color = Color.Yellow)
                    Spacer(Modifier.width(3.dp))
                }
                // AGC
                sat.agcDb?.let {
                    Text(
                        "AGC:%.1f".format(it),
                        fontFamily = FontFamily.Monospace,
                        fontSize = 9.sp,
                        color = Color(0xFF888888)
                    )
                }
            }
        }
    }
}
