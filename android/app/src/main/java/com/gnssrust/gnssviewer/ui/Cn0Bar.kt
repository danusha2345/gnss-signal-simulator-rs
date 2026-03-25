package com.gnssrust.gnssviewer.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.unit.dp
import com.gnssrust.gnssviewer.ui.theme.Cn0Medium
import com.gnssrust.gnssviewer.ui.theme.Cn0Strong
import com.gnssrust.gnssviewer.ui.theme.Cn0Weak
import com.gnssrust.gnssviewer.ui.theme.DarkSurfaceVariant

@Composable
fun Cn0Bar(cn0: Float, modifier: Modifier = Modifier) {
    val maxCn0 = 50f
    val fraction = (cn0 / maxCn0).coerceIn(0f, 1f)
    val color = when {
        cn0 >= 40f -> Cn0Strong
        cn0 >= 25f -> Cn0Medium
        else -> Cn0Weak
    }
    val shape = RoundedCornerShape(3.dp)
    Box(
        modifier = modifier
            .height(8.dp)
            .clip(shape)
            .background(DarkSurfaceVariant)
    ) {
        Box(
            modifier = Modifier
                .fillMaxHeight()
                .fillMaxWidth(fraction)
                .background(color, shape)
        )
    }
}
