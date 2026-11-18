# ATEM Software Control USB capture for polling-discipline analysis.
# MUST run elevated (UAC prompt will appear).
#
#   powershell -ExecutionPolicy Bypass -File capture_asc.ps1 -Seconds 150
#
# What it does:
#   1. Removes the ATEM composite device so Windows re-enumerates it and the
#      Blackmagic driver package rebinds (Zadig's WinUSB binding is dropped).
#   2. Starts USBPcap on all four root hubs (one file each).
#   3. Launches ATEM Software Control.
#   4. Waits, then stops the captures.
#
# Afterwards: the pcap containing the ATEM traffic is identified by scanning
# for the Blackmagic VID bytes (0x1e 0xdb).

param(
    [int]$Seconds = 150,
    [string]$OutDir = "C:\Users\Flowing\embassy-usb-bmd-atem\captures\asc"
)

$ErrorActionPreference = 'Continue'
$log = Join-Path $OutDir "run.log"
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
function Log($m) {
    $line = "[{0:HH:mm:ss}] {1}" -f (Get-Date), $m
    Add-Content -Path $log -Value $line
    Write-Host $line
}

Log "=== elevated run started (admin=$(([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator))) ==="

# --- 1. restore the Blackmagic driver binding -------------------------------
$composite = "USB\VID_1EDB&PID_BE49\4AB85854F7D64F149EDDEC7FAE2403D6"
Log "removing composite device to force re-enumeration…"
& pnputil /remove-device $composite 2>&1 | ForEach-Object { Log "  $_" }
Start-Sleep -Seconds 3
Log "scanning for hardware changes…"
& pnputil /scan-devices 2>&1 | ForEach-Object { Log "  $_" }
Start-Sleep -Seconds 6

Log "current ATEM device tree:"
Get-PnpDevice -ErrorAction SilentlyContinue |
    Where-Object { $_.InstanceId -like "*VID_1EDB*" } |
    Select-Object Status, Class, InstanceId |
    Format-Table -AutoSize | Out-String -Width 200 |
    ForEach-Object { Log $_ }

# --- 2. start USBPcap on every root hub -------------------------------------
$caps = @()
foreach ($n in 1..4) {
    $out = Join-Path $OutDir "hub$n.pcap"
    Remove-Item $out -ErrorAction SilentlyContinue
    $p = Start-Process -FilePath "C:\Program Files\USBPcap\USBPcapCMD.exe" `
        -ArgumentList '-d', "\\.\USBPcap$n", '-o', $out, '-A' `
        -PassThru -NoNewWindow -RedirectStandardError (Join-Path $OutDir "hub$n.err")
    $caps += [pscustomobject]@{ N = $n; Proc = $p; File = $out }
    Log "capture started on USBPcap$n (pid $($p.Id))"
}
Start-Sleep -Seconds 1

# --- 3. launch ATEM Software Control ----------------------------------------
$asc = "C:\Program Files (x86)\Blackmagic Design\Blackmagic ATEM Switchers\ATEM Software Control\ATEM Software Control.exe"
if (Test-Path $asc) {
    Log "launching ATEM Software Control…"
    Start-Process -FilePath $asc
} else {
    Log "WARNING: ATEM Software Control not found at $asc"
}

Log ">>> CONNECT ATEM Software Control TO THE USB DEVICE AND PRESS BUTTONS NOW <<<"
Log "waiting $Seconds s…"
Start-Sleep -Seconds $Seconds

# --- 4. stop captures -------------------------------------------------------
foreach ($c in $caps) {
    if (-not $c.Proc.HasExited) { $c.Proc.Kill() }
    Start-Sleep -Milliseconds 400
}
Start-Sleep -Seconds 1
Log "capture files:"
Get-ChildItem (Join-Path $OutDir "*.pcap") -ErrorAction SilentlyContinue |
    Select-Object Name, Length |
    Format-Table -AutoSize | Out-String -Width 120 |
    ForEach-Object { Log $_ }
Log "=== done ==="
