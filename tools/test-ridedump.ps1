$ErrorActionPreference = 'Stop'

function Get-ApexCrc32 {
    param([byte[]]$Data)
    [uint32]$crc = [uint32]::MaxValue
    [uint32]$polynomial = 3988292384
    foreach ($byte in $Data) {
        $crc = [uint32]($crc -bxor $byte)
        for ($bit = 0; $bit -lt 8; $bit++) {
            $crc = if (($crc -band 1) -ne 0) {
                [uint32](($crc -shr 1) -bxor $polynomial)
            } else { [uint32]($crc -shr 1) }
        }
    }
    return [uint32]($crc -bxor [uint32]::MaxValue)
}

function Assert-Contains {
    param([string]$Path, [string]$Expected)
    if (-not (Select-String -LiteralPath $Path -SimpleMatch $Expected -Quiet)) {
        throw "$Path did not contain: $Expected"
    }
}

$scratch = Join-Path ([IO.Path]::GetTempPath()) ('apexride-ridedump-' + [guid]::NewGuid())
[IO.Directory]::CreateDirectory($scratch) | Out-Null

try {
    $headerStream = [IO.MemoryStream]::new()
    $header = [IO.BinaryWriter]::new($headerStream)
    $header.Write([uint32]0x31445241); $header.Write([uint16]1); $header.Write([uint16]32)
    $header.Write([uint32]42); $header.Write([uint32]1700000000); $header.Write([uint32]1000)
    $header.Write([uint16]0x0104); $header.Write([uint16]3); $header.Write([uint16]50)
    $header.Write([uint16]0); $header.Flush()
    $headerBytes = $headerStream.ToArray()

    $ridePath = Join-Path $scratch 'R000042.bin'
    $rideStream = [IO.MemoryStream]::new()
    $writer = [IO.BinaryWriter]::new($rideStream)
    $writer.Write($headerBytes); $writer.Write((Get-ApexCrc32 $headerBytes))

    $writer.Write([byte]1); $writer.Write([byte]22); $writer.Write([uint32]1100)
    foreach ($value in [int16[]]@(1234, -250, 0, 100, -200, 1000, 15, -25, 35)) { $writer.Write($value) }

    $writer.Write([byte]2); $writer.Write([byte]26); $writer.Write([uint32]1200)
    $writer.Write([uint32]1700000001); $writer.Write([int32]-62000000); $writer.Write([int32]1068000000)
    $writer.Write([uint16]1000); $writer.Write([uint16]12345); $writer.Write([int16]125)
    $writer.Write([byte]15); $writer.Write([byte]8); $writer.Write([byte]3); $writer.Write([byte]0)

    $writer.Write([byte]3); $writer.Write([byte]10); $writer.Write([uint32]1000)
    $writer.Write([byte]1); $writer.Write([byte]0); $writer.Write([int32]0)
    $writer.Flush()
    $rideBytes = $rideStream.ToArray()
    $writer.Dispose()
    $rideStream.Dispose()
    [IO.File]::WriteAllBytes($ridePath, $rideBytes)

    $tool = Join-Path $PSScriptRoot 'ridedump.ps1'
    $imu = Join-Path $scratch 'imu.csv'; $gnss = Join-Path $scratch 'gnss.csv'
    $gpx = Join-Path $scratch 'ride.gpx'; $events = Join-Path $scratch 'events.csv'
    $report = Join-Path $scratch 'report.txt'

    & pwsh -NoLogo -NoProfile -File $tool $ridePath Report -OutputPath $report
    if ($LASTEXITCODE -ne 0) { throw 'report generation failed' }

    & pwsh -NoLogo -NoProfile -File $tool $ridePath ImuCsv -OutputPath $imu
    if ($LASTEXITCODE -ne 0) { throw 'IMU CSV export failed' }
    & pwsh -NoLogo -NoProfile -File $tool $ridePath GnssCsv -OutputPath $gnss
    if ($LASTEXITCODE -ne 0) { throw 'GNSS CSV export failed' }
    & pwsh -NoLogo -NoProfile -File $tool $ridePath Gpx -OutputPath $gpx
    if ($LASTEXITCODE -ne 0) { throw 'GPX export failed' }
    & pwsh -NoLogo -NoProfile -File $tool $ridePath Events -OutputPath $events
    if ($LASTEXITCODE -ne 0) { throw 'event export failed' }

    Assert-Contains $imu '1100,12.34,-2.50,0.00,0.100,-0.200,1.000,1.5,-2.5,3.5'
    Assert-Contains $gnss '1200,1700000001,-6.2000000,106.8000000,36.00,123.45,12.5,1.5,8,3'
    Assert-Contains $gpx '<trkpt lat="-6.2000000" lon="106.8000000"><ele>12.5</ele><time>2023-11-14T22:13:21Z</time></trkpt>'
    Assert-Contains $events '1000,RideStart,0'
    Assert-Contains $report 'Ride       R000042'
    Assert-Contains $report 'header CRC OK'
    Assert-Contains $report 'Records    1 IMU, 1 GNSS (1 fixed), 1 events, 0 unknown'

    $badCrcPath = Join-Path $scratch 'bad-crc.bin'
    $badCrcBytes = [byte[]]$rideBytes.Clone()
    $badCrcBytes[28] = $badCrcBytes[28] -bxor 0x01
    [IO.File]::WriteAllBytes($badCrcPath, $badCrcBytes)
    & pwsh -NoLogo -NoProfile -File $tool $badCrcPath Report -OutputPath (Join-Path $scratch 'bad-crc.txt')
    if ($LASTEXITCODE -eq 0) { throw 'bad header CRC was accepted' }

    $truncatedPath = Join-Path $scratch 'truncated.bin'
    [IO.File]::WriteAllBytes($truncatedPath, $rideBytes[0..($rideBytes.Length - 2)])
    & pwsh -NoLogo -NoProfile -File $tool $truncatedPath Report -OutputPath (Join-Path $scratch 'truncated.txt')
    if ($LASTEXITCODE -eq 0) { throw 'truncated ride stream was accepted' }

    Write-Host 'PASS: Windows ride decoder report, exports, CRC rejection and truncation detection validated.'
} finally {
    if (Test-Path -LiteralPath $scratch) {
        Remove-Item -LiteralPath $scratch -Recurse -Force -ErrorAction SilentlyContinue
    }
}
