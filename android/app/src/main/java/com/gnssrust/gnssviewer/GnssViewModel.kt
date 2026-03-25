package com.gnssrust.gnssviewer

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.gnssrust.gnssviewer.model.*
import kotlinx.coroutines.flow.*

class GnssViewModel(application: Application) : AndroidViewModel(application) {

    private val provider = GnssDataProvider(application)

    val satellites: StateFlow<List<SatelliteInfo>> = provider.satellites
    val fixInfo: StateFlow<FixInfo> = provider.fixInfo

    private val _sortOption = MutableStateFlow(SortOption.BY_CN0)
    val sortOption: StateFlow<SortOption> = _sortOption.asStateFlow()

    val sortedSatellites: StateFlow<List<SatelliteInfo>> = combine(
        satellites, _sortOption
    ) { sats, sort ->
        applySorting(sats, sort)
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), emptyList())

    val systemCounts: StateFlow<Map<ConstellationType, Int>> = satellites.map { sats ->
        sats.groupBy { it.constellation }
            .mapValues { it.value.size }
            .toSortedMap(compareBy { it.sortOrder })
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), emptyMap())

    val usedInFixCount: StateFlow<Int> = satellites.map { sats ->
        sats.count { it.usedInFix }
    }.stateIn(viewModelScope, SharingStarted.WhileSubscribed(5000), 0)

    fun setSortOption(option: SortOption) {
        _sortOption.value = option
    }

    fun startListening() = provider.start()
    fun stopListening() = provider.stop()

    private fun applySorting(
        sats: List<SatelliteInfo>,
        sort: SortOption
    ): List<SatelliteInfo> = when (sort) {
        SortOption.BY_SYSTEM -> sats.sortedWith(
            compareBy<SatelliteInfo> { it.constellation.sortOrder }
                .thenBy { it.svid }
                .thenBy { it.band?.name ?: "ZZZ" }
        )
        SortOption.BY_SVID -> sats.sortedWith(
            compareBy<SatelliteInfo> { it.svid }
                .thenBy { it.constellation.sortOrder }
        )
        SortOption.BY_CN0 -> sats.sortedByDescending { it.cn0DbHz }
        SortOption.BY_ELEVATION -> sats.sortedByDescending { it.elevationDeg }
        SortOption.BY_AZIMUTH -> sats.sortedBy { it.azimuthDeg }
        SortOption.BY_USED_IN_FIX -> sats.sortedWith(
            compareByDescending<SatelliteInfo> { it.usedInFix }
                .thenByDescending { it.cn0DbHz }
        )
        SortOption.BY_BAND -> sats.sortedWith(
            compareBy<SatelliteInfo> { it.band?.name ?: "ZZZ" }
                .thenBy { it.constellation.sortOrder }
                .thenBy { it.svid }
        )
    }
}
