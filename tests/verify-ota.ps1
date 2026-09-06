param(
  [ValidateSet("esp32", "esp32s3", "all")]
  [string]$Target = "esp32"
)

$ErrorActionPreference = "Stop"

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$sketchDirectory = Join-Path $PSScriptRoot ".arduino-sketch\ESP32-Irrigation"
$sketchPath = Join-Path $sketchDirectory "ESP32-Irrigation.ino"
$sourcePath = Join-Path $repositoryRoot "firmware\ESP32-Irrigation\ESP32-Irrigation.ino"

$arduinoCommand = Get-Command arduino-cli -ErrorAction SilentlyContinue
if ($arduinoCommand) {
  $arduinoCli = $arduinoCommand.Source
} else {
  $localAppData = [Environment]::GetFolderPath("LocalApplicationData")
  $arduinoCli = Join-Path $localAppData "Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
}

if (-not (Test-Path -LiteralPath $arduinoCli)) {
  throw "arduino-cli was not found. Install Arduino IDE 2 or add arduino-cli to PATH."
}

New-Item -ItemType Directory -Path $sketchDirectory -Force | Out-Null
Copy-Item -LiteralPath $sourcePath -Destination $sketchPath -Force

$targets = @()
if ($Target -eq "esp32" -or $Target -eq "all") {
  $targets += [pscustomobject]@{
    Name = "ESP32 Dev Module"
    Fqbn = "esp32:esp32:esp32:PartitionScheme=min_spiffs"
  }
}
if ($Target -eq "esp32s3" -or $Target -eq "all") {
  $targets += [pscustomobject]@{
    Name = "ESP32-S3 Dev Module"
    Fqbn = "esp32:esp32:esp32s3:PartitionScheme=min_spiffs"
  }
}

foreach ($board in $targets) {
  Write-Host "Compiling $($board.Name) with Minimal SPIFFS (OTA)..."
  & $arduinoCli compile --fqbn $board.Fqbn --jobs 0 $sketchDirectory
  if ($LASTEXITCODE -ne 0) {
    throw "$($board.Name) OTA build failed with exit code $LASTEXITCODE."
  }
}

Write-Host "OTA build verification passed. Subsequent runs reuse Arduino's persistent cache."
