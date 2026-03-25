package com.gnssrust.gnssviewer.ui

import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.gnssrust.gnssviewer.GnssViewModel
import com.gnssrust.gnssviewer.model.SatelliteInfo

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun SatelliteListScreen(viewModel: GnssViewModel) {
    val satellites by viewModel.sortedSatellites.collectAsStateWithLifecycle()
    val fixInfo by viewModel.fixInfo.collectAsStateWithLifecycle()
    val systemCounts by viewModel.systemCounts.collectAsStateWithLifecycle()
    val usedInFix by viewModel.usedInFixCount.collectAsStateWithLifecycle()
    val sortOption by viewModel.sortOption.collectAsStateWithLifecycle()

    var selectedSatellite by remember { mutableStateOf<SatelliteInfo?>(null) }

    // Detail dialog
    selectedSatellite?.let { sat ->
        SatelliteDetailSheet(
            sat = sat,
            onDismiss = { selectedSatellite = null }
        )
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("GNSS Viewer") },
                colors = TopAppBarDefaults.topAppBarColors(
                    containerColor = MaterialTheme.colorScheme.surface
                )
            )
        }
    ) { padding ->
        Column(modifier = Modifier.padding(padding)) {
            StatusHeader(fixInfo, systemCounts, usedInFix)

            SortBar(sortOption) { viewModel.setSortOption(it) }

            // Column headers
            SatelliteListHeader()

            HorizontalDivider(
                color = Color.DarkGray,
                thickness = 0.5.dp,
                modifier = Modifier.padding(horizontal = 4.dp)
            )

            // Satellite list
            LazyColumn(
                modifier = Modifier.fillMaxSize()
            ) {
                items(
                    items = satellites,
                    key = { it.key }
                ) { sat ->
                    SatelliteRow(
                        sat = sat,
                        onClick = { selectedSatellite = sat }
                    )
                }
            }
        }
    }
}

@Composable
private fun SatelliteListHeader() {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp, vertical = 2.dp)
    ) {
        Text("Sys", Modifier.width(18.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Text("SV", Modifier.width(30.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Spacer(Modifier.width(4.dp))
        Text("Band", Modifier.width(30.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Spacer(Modifier.width(4.dp))
        Text("CN0", Modifier.width(56.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Spacer(Modifier.width(4.dp))
        Text("dBHz", Modifier.width(38.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Text("El", Modifier.width(32.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Text("Az", Modifier.width(36.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
        Text("Dop m/s", Modifier.width(48.dp), fontSize = 10.sp,
            color = Color.Gray, fontFamily = FontFamily.Monospace)
    }
}
