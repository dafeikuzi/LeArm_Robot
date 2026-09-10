param(
    [string]$SerialPort = "auto",
    [int]$CameraIndex = 0,
    [ValidateSet("auto", "msmf", "dshow")]
    [string]$CameraBackend = "auto",
    [string]$BindHost = "",
    [int]$SerialTcpPort = 8766,
    [int]$CameraTcpPort = 8767,
    [switch]$ListDevices
)

$ErrorActionPreference = "Stop"
$BridgeRoot = Join-Path $env:LOCALAPPDATA "LeArmBridge"
$VenvPython = Join-Path $BridgeRoot "venv\Scripts\python.exe"
$BridgeScript = Join-Path $PSScriptRoot "windows_hardware_bridge.py"
$Requirements = Join-Path $PSScriptRoot "windows_bridge_requirements.txt"

if (-not (Test-Path $VenvPython)) {
    New-Item -ItemType Directory -Force -Path $BridgeRoot | Out-Null
    py -3.13 -m venv (Join-Path $BridgeRoot "venv")
}

$PreviousNoProxy = $env:NO_PROXY
$MirrorNoProxy = "pypi.tuna.tsinghua.edu.cn"
$env:NO_PROXY = if ($PreviousNoProxy) {
    "$PreviousNoProxy,$MirrorNoProxy"
} else {
    $MirrorNoProxy
}
try {
    & $VenvPython -m pip install --disable-pip-version-check `
        --timeout 30 --retries 5 `
        -i https://pypi.tuna.tsinghua.edu.cn/simple `
        -r $Requirements
    if ($LASTEXITCODE -ne 0) {
        throw "Dependency installation failed with exit code $LASTEXITCODE."
    }
} finally {
    $env:NO_PROXY = $PreviousNoProxy
}

if ($ListDevices) {
    & $VenvPython $BridgeScript --list-devices
    exit $LASTEXITCODE
}

if (-not $BindHost) {
    $RouteOutput = @(& wsl.exe --exec /usr/sbin/ip route show default)
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot query the WSL default route (exit code $LASTEXITCODE)."
    }
    $RouteText = ($RouteOutput -join " ").Trim()
    if ($RouteText -match 'default\s+via\s+([^\s]+)') {
        $BindHost = $Matches[1]
    }
}
if (-not $BindHost) {
    throw "Cannot determine the Windows host address for WSL; pass -BindHost explicitly."
}

$AddressOutput = @(& wsl.exe --exec /usr/bin/hostname -I)
if ($LASTEXITCODE -ne 0) {
    throw "Cannot query the WSL address (exit code $LASTEXITCODE)."
}
$AddressParts = @(($AddressOutput -join " ") -split '\s+' | Where-Object { $_ })
$WslAddress = if ($AddressParts.Count -gt 0) { $AddressParts[0] } else { $null }
if (-not $WslAddress) {
    throw "Cannot determine the current WSL address for the firewall rule."
}

$FirewallRuleName = "LeArm WSL TCP Hardware Bridge"
try {
    Get-NetFirewallRule -DisplayName $FirewallRuleName -ErrorAction SilentlyContinue |
        Remove-NetFirewallRule -ErrorAction Stop
    New-NetFirewallRule -DisplayName $FirewallRuleName `
        -Direction Inbound -Action Allow -Protocol TCP `
        -LocalAddress $BindHost -LocalPort @($SerialTcpPort, $CameraTcpPort) `
        -RemoteAddress $WslAddress -Profile Any | Out-Null
} catch {
    throw "Cannot create the firewall rule. Run this script in an Administrator PowerShell. Error: $($_.Exception.Message)"
}

$Arguments = @(
    $BridgeScript,
    "--serial-port", $SerialPort,
    "--camera-index", $CameraIndex,
    "--camera-backend", $CameraBackend,
    "--bind-host", $BindHost,
    "--serial-tcp-port", $SerialTcpPort,
    "--camera-tcp-port", $CameraTcpPort
)

Write-Host "Windows bridge address: $BindHost"
Write-Host "Serial TCP: $BindHost`:$SerialTcpPort"
Write-Host "Camera TCP: $BindHost`:$CameraTcpPort"
Write-Host "Firewall remote address: $WslAddress"
& $VenvPython @Arguments
exit $LASTEXITCODE
