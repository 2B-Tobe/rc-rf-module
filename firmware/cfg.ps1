# cfg.ps1 - send a setup command to the RF module over its log UART (J3: adapter TX -> PA06, adapter RX <- PA05)
# and show the reply.  Close PuTTY first: only one program can hold the COM port.
#
#   .\cfg.ps1 -Port COM5                          show the stored setup and what runs now  (= "cfg")
#   .\cfg.ps1 -Port COM5 -Command "cfg rx rlink"  store: receiver, RLINK
#   .\cfg.ps1 -Port COM5 -Command "cfg rx dsmx 5sbus"   receiver, DSMX, SBUS on servo connector 6 (8pwm 5sbus 5ppm 5both)
#   .\cfg.ps1 -Port COM5 -Command "cfg tx rlink dual"
#   .\cfg.ps1 -Port COM5 -Command "cfg clear"
#   .\cfg.ps1 -Ports                              list the COM ports
#   -All  shows every line the module prints while waiting (default: only CFG / SETUP / BAY / JUMPERS lines)
param(
  [string]$Port = "",
  [string]$Command = "cfg",
  [int]$Wait = 4,
  [switch]$All,
  [switch]$Ports
)
if ($Ports -or $Port -eq "") {
  Write-Host "COM ports:"
  Get-CimInstance Win32_SerialPort | ForEach-Object { Write-Host ("  {0}  {1}" -f $_.DeviceID, $_.Description) }
  if ($Port -eq "") { Write-Host "Use: .\cfg.ps1 -Port COMn -Command `"cfg rx rlink`""; exit }
}
$p = New-Object System.IO.Ports.SerialPort $Port, 115200, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$p.Handshake = [System.IO.Ports.Handshake]::None
$p.DtrEnable = $false
$p.RtsEnable = $false
try { $p.Open() } catch { Write-Host "Cannot open $Port : $($_.Exception.Message)  (PuTTY still open?)"; exit 1 }
Start-Sleep -Milliseconds 300
$null = $p.ReadExisting()
$p.Write($Command + "`r")
Write-Host ("> " + $Command)
$end = (Get-Date).AddSeconds($Wait)
$buf = ""
while ((Get-Date) -lt $end) {
  $s = $p.ReadExisting()
  if ($s.Length -gt 0) { $buf += $s }
  Start-Sleep -Milliseconds 100
}
$p.Close()
$lines = $buf -split "`r?`n"
$shown = 0
foreach ($l in $lines) {
  $t = $l.Trim()
  if ($t.Length -eq 0) { continue }
  if ($All -or $t -match '^(CFG|SETUP|BAY|JUMPERS|RESET)') { Write-Host $t; $shown++ }
}
if ($shown -eq 0) { Write-Host "No reply. Is the adapter's TX wire on J3 RX (PA06) and its RX on J3 TX (PA05)? Try -All to see everything received." }
