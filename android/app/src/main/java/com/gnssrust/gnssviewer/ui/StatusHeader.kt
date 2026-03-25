package com.gnssrust.gnssviewer.ui

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.gnssrust.gnssviewer.model.ConstellationType
import com.gnssrust.gnssviewer.model.FixInfo

@Composable
fun StatusHeader(
    fixInfo: FixInfo,
    systemCounts: Map<ConstellationType, Int>,
    usedInFix: Int
) {
    Surface(
        color = MaterialTheme.colorScheme.surfaceVariant,
        shape = RoundedCornerShape(8.dp),
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 8.dp, vertical = 4.dp)
    ) {
        Column(modifier = Modifier.padding(10.dp)) {
            if (fixInfo.hasFix) {
                // Row 1: coordinates
                Text(
                    text = "${formatLat(fixInfo.latitude)}  ${formatLon(fixInfo.longitude)}  " +
                            "▲${fixInfo.altitude.toInt()}m",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 13.sp,
                    color = Color.White
                )
                // Row 2: accuracies
                val accParts = mutableListOf("H:±${fixInfo.accuracy.toInt()}m")
                fixInfo.verticalAccuracy?.let { accParts.add("V:±${it.toInt()}m") }
                if (fixInfo.speed > 0.5f) {
                    accParts.add("%.1f m/s".format(fixInfo.speed))
                }
                fixInfo.speedAccuracy?.let {
                    if (fixInfo.speed > 0.5f) accParts.add("±%.1f".format(it))
                }
                Text(
                    text = accParts.joinToString("  "),
                    fontFamily = FontFamily.Monospace,
                    fontSize = 11.sp,
                    color = Color.Gray
                )
            } else {
                Text(
                    text = "Waiting for fix...",
                    fontSize = 13.sp,
                    color = Color.Gray
                )
            }

            Spacer(Modifier.height(4.dp))

            // Row 3: system counts
            Row {
                systemCounts.forEach { (sys, count) ->
                    Text(
                        text = "${sys.shortName}:$count",
                        color = Color(sys.colorArgb),
                        fontFamily = FontFamily.Monospace,
                        fontSize = 13.sp
                    )
                    Spacer(Modifier.width(8.dp))
                }
                val total = systemCounts.values.sum()
                Text(
                    text = "= $total",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 13.sp,
                    color = Color.White
                )
                Spacer(Modifier.width(12.dp))
                Text(
                    text = "★$usedInFix",
                    fontFamily = FontFamily.Monospace,
                    fontSize = 13.sp,
                    color = Color.Yellow
                )
            }
        }
    }
}

private fun formatLat(lat: Double): String {
    val dir = if (lat >= 0) "N" else "S"
    return "%.4f°$dir".format(Math.abs(lat))
}

private fun formatLon(lon: Double): String {
    val dir = if (lon >= 0) "E" else "W"
    return "%.4f°$dir".format(Math.abs(lon))
}
