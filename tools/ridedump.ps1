[CmdletBinding()]
param(
    [Parameter(Mandatory, Position = 0)]
    [ValidateScript({ Test-Path -LiteralPath $_ -PathType Leaf })]
    [string]$Path,

    [Parameter(Position = 1)]
    [ValidateSet('Report', 'ImuCsv', 'GnssCsv', 'Gpx', 'Events')]
    [string]$Mode = 'Report',

    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'
$culture = [Globalization.CultureInfo]::InvariantCulture
$eventNames = @{
    1 = 'RideStart'; 2 = 'RideEnd'; 3 = 'GnssFixAcquired'; 4 = 'GnssFixLost'
    5 = 'CalibrationApplied'; 6 = 'StorageLow'; 7 = 'RecordingPaused'
    8 = 'RecordingResumed'; 9 = 'BufferOverrun'; 10 = 'StorageFull'
}

function Get-ApexCrc32 {
    param([byte[]]$Data, [int]$Offset, [int]$Count)

    [uint32]$crc = [uint32]::MaxValue
    [uint32]$polynomial = 3988292384
    for ($i = $Offset; $i -lt $Offset + $Count; $i++) {
        $crc = [uint32]($crc -bxor $Data[$i])
        for ($bit = 0; $bit -lt 8; $bit++) {
            if (($crc -band 1) -ne 0) {
                $crc = [uint32](($crc -shr 1) -bxor $polynomial)
            } else {
                $crc = [uint32]($crc -shr 1)
            }
        }
    }
    return [uint32]($crc -bxor [uint32]::MaxValue)
}

function Format-Invariant {
    param([string]$Template, [object[]]$Values)
    return [string]::Format($culture, $Template, $Values)
}

$bytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Path))
if ($bytes.Length -lt 32) {
    throw "The file is too short to contain an ApexRide header."
}

$stream = [IO.MemoryStream]::new($bytes, $false)
$reader = [IO.BinaryReader]::new($stream)
$ownedWriter = $null
$writer = [Console]::Out

try {
    if ($OutputPath) {
        $absoluteOutput = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputPath)
        $parent = Split-Path -Parent $absoluteOutput
        if ($parent -and -not (Test-Path -LiteralPath $parent)) {
            [IO.Directory]::CreateDirectory($parent) | Out-Null
        }
        $ownedWriter = [IO.StreamWriter]::new($absoluteOutput, $false, [Text.UTF8Encoding]::new($false))
        $writer = $ownedWriter
    }

    $magic = $reader.ReadUInt32()
    $formatVersion = $reader.ReadUInt16()
    $headerSize = $reader.ReadUInt16()
    $rideId = $reader.ReadUInt32()
    $startUnixTime = $reader.ReadUInt32()
    $startMillis = $reader.ReadUInt32()
    $firmwareVersion = $reader.ReadUInt16()
    $calibrationVersion = $reader.ReadUInt16()
    $imuLogRateHz = $reader.ReadUInt16()
    $gnssLogRateHz = $reader.ReadUInt16()
    $storedHeaderCrc = $reader.ReadUInt32()
    $calculatedHeaderCrc = Get-ApexCrc32 -Data $bytes -Offset 0 -Count 28

    $headerValid = $magic -eq 0x31445241 -and $formatVersion -eq 1 -and
                   $headerSize -ge 32 -and $headerSize -le $bytes.Length -and
                   $storedHeaderCrc -eq $calculatedHeaderCrc
    if ($headerSize -lt 32 -or $headerSize -gt $bytes.Length) {
        throw "The ApexRide header size is invalid."
    }
    $stream.Position = $headerSize

    if ($Mode -eq 'ImuCsv') {
        $writer.WriteLine('timestamp_ms,roll_deg,pitch_deg,yaw_deg,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps')
    } elseif ($Mode -eq 'GnssCsv') {
        $writer.WriteLine('timestamp_ms,unix_time,latitude,longitude,speed_kmh,heading_deg,altitude_m,hdop,satellites,fix')
    } elseif ($Mode -eq 'Gpx') {
        $writer.WriteLine('<?xml version="1.0" encoding="UTF-8"?>')
        $writer.WriteLine('<gpx version="1.1" creator="ApexRide ridedump" xmlns="http://www.topografix.com/GPX/1/1">')
        $writer.WriteLine((Format-Invariant '  <trk><name>R{0:D6}</name><trkseg>' @($rideId)))
    } elseif ($Mode -eq 'Events') {
        $writer.WriteLine('timestamp_ms,event,value')
    }

    [uint32]$imuCount = 0
    [uint32]$gnssCount = 0
    [uint32]$fixCount = 0
    [uint32]$eventCount = 0
    [uint32]$unknownCount = 0
    $truncated = $false

    while ($stream.Position + 2 -le $stream.Length) {
        $type = $reader.ReadByte()
        $length = $reader.ReadByte()
        if ($stream.Position + $length -gt $stream.Length) {
            $truncated = $true
            break
        }

        $payloadStart = $stream.Position
        if ($type -eq 1 -and $length -eq 22) {
            $timestamp = $reader.ReadUInt32()
            $roll = $reader.ReadInt16(); $pitch = $reader.ReadInt16(); $yaw = $reader.ReadInt16()
            $ax = $reader.ReadInt16(); $ay = $reader.ReadInt16(); $az = $reader.ReadInt16()
            $gx = $reader.ReadInt16(); $gy = $reader.ReadInt16(); $gz = $reader.ReadInt16()
            $imuCount++
            if ($Mode -eq 'ImuCsv') {
                $writer.WriteLine((Format-Invariant '{0},{1:F2},{2:F2},{3:F2},{4:F3},{5:F3},{6:F3},{7:F1},{8:F1},{9:F1}' @(
                    $timestamp, ($roll / 100.0), ($pitch / 100.0), ($yaw / 100.0),
                    ($ax / 1000.0), ($ay / 1000.0), ($az / 1000.0),
                    ($gx / 10.0), ($gy / 10.0), ($gz / 10.0))))
            }
        } elseif ($type -eq 2 -and $length -eq 26) {
            $timestamp = $reader.ReadUInt32(); $unixTime = $reader.ReadUInt32()
            $latitude = $reader.ReadInt32() / 1e7; $longitude = $reader.ReadInt32() / 1e7
            $speedKmh = $reader.ReadUInt16() * 0.036; $heading = $reader.ReadUInt16() / 100.0
            $altitude = $reader.ReadInt16() / 10.0; $hdop = $reader.ReadByte() / 10.0
            $satellites = $reader.ReadByte(); $fixType = $reader.ReadByte(); $null = $reader.ReadByte()
            $gnssCount++
            if ($fixType -ne 0) { $fixCount++ }

            if ($Mode -eq 'GnssCsv') {
                $writer.WriteLine((Format-Invariant '{0},{1},{2:F7},{3:F7},{4:F2},{5:F2},{6:F1},{7:F1},{8},{9}' @(
                    $timestamp, $unixTime, $latitude, $longitude, $speedKmh,
                    $heading, $altitude, $hdop, $satellites, $fixType)))
            } elseif ($Mode -eq 'Gpx' -and $fixType -ne 0) {
                $point = Format-Invariant '    <trkpt lat="{0:F7}" lon="{1:F7}"><ele>{2:F1}</ele>' @(
                    $latitude, $longitude, $altitude)
                if ($unixTime -ne 0) {
                    $stamp = [DateTimeOffset]::FromUnixTimeSeconds($unixTime).UtcDateTime.ToString(
                        'yyyy-MM-ddTHH:mm:ssZ', $culture)
                    $point += "<time>$stamp</time>"
                }
                $writer.WriteLine($point + '</trkpt>')
            }
        } elseif ($type -eq 3 -and $length -eq 10) {
            $timestamp = $reader.ReadUInt32(); $code = $reader.ReadByte()
            $null = $reader.ReadByte(); $value = $reader.ReadInt32()
            $eventCount++
            if ($Mode -eq 'Events') {
                $name = if ($eventNames.ContainsKey([int]$code)) { $eventNames[[int]$code] } else { 'Unknown' }
                $writer.WriteLine((Format-Invariant '{0},{1},{2}' @($timestamp, $name, $value)))
            }
        } else {
            $unknownCount++
        }

        $stream.Position = $payloadStart + $length
    }

    if ($stream.Position -ne $stream.Length) { $truncated = $true }
    if ($Mode -eq 'Gpx') { $writer.WriteLine('  </trkseg></trk>'); $writer.WriteLine('</gpx>') }

    if ($Mode -eq 'Report') {
        $magicText = [Text.Encoding]::ASCII.GetString($bytes, 0, 4)
        $started = if ($startUnixTime -eq 0) { '(no GNSS time)' } else {
            [DateTimeOffset]::FromUnixTimeSeconds($startUnixTime).UtcDateTime.ToString('yyyy-MM-dd HH:mm:ss UTC', $culture)
        }
        $writer.WriteLine("File       $((Resolve-Path -LiteralPath $Path).Path) ($($bytes.Length) bytes)")
        $writer.WriteLine((Format-Invariant 'Magic      {0}  format v{1}  firmware 0x{2:X4}  header CRC {3}' @(
            $magicText, $formatVersion, $firmwareVersion, $(if ($headerValid) { 'OK' } else { 'INVALID' }))))
        $writer.WriteLine((Format-Invariant 'Ride       R{0:D6}' @($rideId)))
        $writer.WriteLine("Started    $started (device clock $startMillis ms)")
        $writer.WriteLine("Log rate   $imuLogRateHz Hz IMU  calibration v$calibrationVersion")
        $writer.WriteLine("Records    $imuCount IMU, $gnssCount GNSS ($fixCount fixed), $eventCount events, $unknownCount unknown")
        $writer.WriteLine("Stream     $(if ($truncated) { 'TRUNCATED' } else { 'complete record boundary' })")
    }

    $writer.Flush()
    if ($OutputPath) { Write-Host "Wrote $absoluteOutput" }
    if (-not $headerValid -or $truncated) { exit 1 }
} finally {
    if ($ownedWriter) { $ownedWriter.Dispose() }
    $reader.Dispose()
    $stream.Dispose()
}
