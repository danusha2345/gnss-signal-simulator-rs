package com.gnssrust.gnssviewer

import android.annotation.SuppressLint
import android.content.Context
import android.location.GnssStatus
import android.location.GnssMeasurement
import android.location.GnssMeasurementsEvent
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import com.gnssrust.gnssviewer.model.*
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow

private data class MeasurementData(
    val dopplerMps: Double,
    val agcDb: Double?,
    val state: TrackingState,
    val multipathIndicator: Int
)

class GnssDataProvider(private val context: Context) {

    private val _satellites = MutableStateFlow<List<SatelliteInfo>>(emptyList())
    val satellites: StateFlow<List<SatelliteInfo>> = _satellites.asStateFlow()

    private val _fixInfo = MutableStateFlow(FixInfo())
    val fixInfo: StateFlow<FixInfo> = _fixInfo.asStateFlow()

    private var latestMeasurements = mapOf<String, MeasurementData>()

    private val locationManager =
        context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    private val handler = Handler(Looper.getMainLooper())

    private val gnssStatusCallback = object : GnssStatus.Callback() {
        override fun onSatelliteStatusChanged(status: GnssStatus) {
            _satellites.value = buildSatelliteList(status)
        }
    }

    private val measurementsCallback = object : GnssMeasurementsEvent.Callback() {
        override fun onGnssMeasurementsReceived(event: GnssMeasurementsEvent) {
            val map = mutableMapOf<String, MeasurementData>()
            for (m in event.measurements) {
                val constType = ConstellationType.fromAndroidType(m.constellationType)
                val freq = getCarrierFreqFromMeasurement(m)
                val band = freq?.let { classifyBand(constType, it) }
                val key = "${constType.shortName}${m.svid}_${band?.name ?: "?"}"
                map[key] = MeasurementData(
                    dopplerMps = m.pseudorangeRateMetersPerSecond,
                    agcDb = getAgcFromMeasurement(m),
                    state = TrackingState(m.state),
                    multipathIndicator = m.multipathIndicator
                )
            }
            latestMeasurements = map
        }
    }

    private val locationListener = object : LocationListener {
        override fun onLocationChanged(location: Location) {
            _fixInfo.value = FixInfo(
                hasFix = true,
                latitude = location.latitude,
                longitude = location.longitude,
                altitude = location.altitude,
                accuracy = location.accuracy,
                verticalAccuracy = getVerticalAccuracy(location),
                speedAccuracy = getSpeedAccuracy(location),
                speed = location.speed,
                bearing = location.bearing,
                satellitesUsed = location.extras?.getInt("satellites", 0) ?: 0,
                timestamp = location.time
            )
        }

        @Deprecated("Deprecated in API 29")
        override fun onStatusChanged(provider: String?, status: Int, extras: Bundle?) {}
        override fun onProviderEnabled(provider: String) {}
        override fun onProviderDisabled(provider: String) {}
    }

    @SuppressLint("MissingPermission")
    fun start() {
        locationManager.registerGnssStatusCallback(gnssStatusCallback, handler)
        locationManager.registerGnssMeasurementsCallback(measurementsCallback, handler)
        locationManager.requestLocationUpdates(
            LocationManager.GPS_PROVIDER,
            1000L,
            0f,
            locationListener,
            Looper.getMainLooper()
        )
    }

    fun stop() {
        locationManager.unregisterGnssStatusCallback(gnssStatusCallback)
        locationManager.unregisterGnssMeasurementsCallback(measurementsCallback)
        locationManager.removeUpdates(locationListener)
    }

    private fun buildSatelliteList(status: GnssStatus): List<SatelliteInfo> {
        val measurements = latestMeasurements
        val list = mutableListOf<SatelliteInfo>()

        for (i in 0 until status.satelliteCount) {
            val constType = ConstellationType.fromAndroidType(status.getConstellationType(i))
            val svid = status.getSvid(i)
            val carrierFreq = getCarrierFreqFromStatus(status, i)
            val band = carrierFreq?.let { classifyBand(constType, it) }
            val key = "${constType.shortName}${svid}_${band?.name ?: "?"}"
            val meas = measurements[key]

            list.add(
                SatelliteInfo(
                    constellation = constType,
                    svid = svid,
                    cn0DbHz = status.getCn0DbHz(i),
                    elevationDeg = status.getElevationDegrees(i),
                    azimuthDeg = status.getAzimuthDegrees(i),
                    hasEphemeris = status.hasEphemerisData(i),
                    hasAlmanac = status.hasAlmanacData(i),
                    usedInFix = status.usedInFix(i),
                    carrierFreqHz = carrierFreq,
                    band = band,
                    basebandCn0DbHz = getBasebandCn0(status, i),
                    dopplerMps = meas?.dopplerMps,
                    agcDb = meas?.agcDb,
                    trackingState = meas?.state,
                    multipathIndicator = meas?.multipathIndicator,
                    statusIndex = i
                )
            )
        }
        return list
    }

    private fun getCarrierFreqFromStatus(status: GnssStatus, index: Int): Double? {
        return if (Build.VERSION.SDK_INT >= 26 && status.hasCarrierFrequencyHz(index)) {
            status.getCarrierFrequencyHz(index).toDouble()
        } else null
    }

    private fun getCarrierFreqFromMeasurement(m: GnssMeasurement): Double? {
        return if (Build.VERSION.SDK_INT >= 26 && m.hasCarrierFrequencyHz()) {
            m.carrierFrequencyHz.toDouble()
        } else null
    }

    @Suppress("DEPRECATION")
    private fun getAgcFromMeasurement(m: GnssMeasurement): Double? {
        return if (Build.VERSION.SDK_INT >= 27 && m.hasAutomaticGainControlLevelDb()) {
            m.automaticGainControlLevelDb
        } else null
    }

    private fun getBasebandCn0(status: GnssStatus, index: Int): Float? {
        return if (Build.VERSION.SDK_INT >= 30 && status.hasBasebandCn0DbHz(index)) {
            status.getBasebandCn0DbHz(index)
        } else null
    }

    private fun getVerticalAccuracy(location: Location): Float? {
        return if (Build.VERSION.SDK_INT >= 26 && location.hasVerticalAccuracy()) {
            location.verticalAccuracyMeters
        } else null
    }

    private fun getSpeedAccuracy(location: Location): Float? {
        return if (Build.VERSION.SDK_INT >= 26 && location.hasSpeedAccuracy()) {
            location.speedAccuracyMetersPerSecond
        } else null
    }
}
