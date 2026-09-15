# =====================================================================
#  Moniteur serie pour le robot balancier
#  - detecte tout seul le port du CH340 / CP210x
#  - horodate chaque ligne et l ecrit dans logs\robot.log
#  - envoie les commandes deposees dans tools\cmd.txt puis vide le fichier
#  - se reconnecte tout seul si la carte est debranchee / reflashee
#
#  Lancement :  powershell -ExecutionPolicy Bypass -File tools\serial_monitor.ps1
#  Arret     :  Ctrl+C  (ou suppression du fichier tools\STOP)
# =====================================================================
param(
    [string]$Port = "",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Continue"

$root    = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$logDir  = Join-Path $root "logs"
$logPath = Join-Path $logDir "robot.log"
$cmdPath = Join-Path $root "tools\cmd.txt"
$stopPath = Join-Path $root "tools\STOP"
$pausePath = Join-Path $root "tools\PAUSE"

if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }
if (Test-Path $stopPath) { Remove-Item $stopPath -Force }

function Find-BoardPort {
    # 1) on cherche un adaptateur USB-serie connu
    $devs = Get-CimInstance Win32_PnPEntity -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -match '\(COM\d+\)' }
    foreach ($d in $devs) {
        if ($d.Name -match 'CH340|CH910|CP210|Silicon Labs|USB-SERIAL|USB Serial|FTDI|UART') {
            if ($d.Name -match '\((COM\d+)\)') { return $Matches[1] }
        }
    }
    # 2) sinon, n importe quel port sauf COM1 (port carte-mere)
    $all = [System.IO.Ports.SerialPort]::GetPortNames() | Where-Object { $_ -ne 'COM1' }
    if ($all) { return ($all | Sort-Object | Select-Object -Last 1) }
    return $null
}

function Write-Log([string]$text) {
    $stamp = (Get-Date).ToString("HH:mm:ss.fff")
    Add-Content -Path $logPath -Value "$stamp $text" -Encoding UTF8
}

Write-Log "=== moniteur demarre ==="

$sp = $null
$buffer = ""
$lastPortMsg = ""
$lastFlashCheck = (Get-Date).AddSeconds(-10)
$flashing = $false

while (-not (Test-Path $stopPath)) {

    # ------------------------------------------- detection auto du flash
    # Si esptool tourne, l IDE est en train de televerser : on lache le
    # port immediatement et on le reprend quand esptool a fini. Evite
    # d avoir a mettre le moniteur en pause a la main a chaque fois.
    # On teste le nom du processus ET sa ligne de commande : selon la version
    # de l IDE, esptool tourne sous des noms varies (esptool.exe, un binaire
    # versionne, ou lance via python). Le seul point commun fiable est la
    # presence de "esptool" quelque part dans la ligne de commande.
    if (((Get-Date) - $lastFlashCheck).TotalMilliseconds -gt 600) {
        $lastFlashCheck = Get-Date
        $flashing = $false
        if (Get-Process -Name esptool* -ErrorAction SilentlyContinue) { $flashing = $true }
        if (-not $flashing) {
            $p = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue |
                 Where-Object { $_.CommandLine -match 'esptool' -and $_.ProcessId -ne $PID }
            if ($p) { $flashing = $true }
        }
    }
    if ($flashing) {
        if ($sp -and $sp.IsOpen) {
            try { $sp.Close() } catch {}
            $sp = $null
            $lastPortMsg = ""
            Write-Log "--- televersement detecte, port libere ---"
        }
        Start-Sleep -Milliseconds 300
        continue
    }

    # ---------------------------------------------------- pause (televersement)
    # Tant que tools\PAUSE existe on lache le port, sinon l IDE Arduino
    # ne peut pas flasher : le port est verrouille en exclusif.
    if (Test-Path $pausePath) {
        if ($sp -and $sp.IsOpen) {
            try { $sp.Close() } catch {}
            $sp = $null
            $lastPortMsg = ""
            Write-Log "--- PAUSE : port libere pour televersement ---"
        }
        Start-Sleep -Milliseconds 400
        continue
    }

    # ---------------------------------------------------- (re)connexion
    if ($null -eq $sp -or -not $sp.IsOpen) {
        $p = $Port
        if (-not $p) { $p = Find-BoardPort }

        if (-not $p) {
            if ($lastPortMsg -ne "none") {
                Write-Log "--- aucun port detecte, attente ---"
                $lastPortMsg = "none"
            }
            Start-Sleep -Milliseconds 1000
            continue
        }

        try {
            $sp = New-Object System.IO.Ports.SerialPort $p, $Baud, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
            # IMPORTANT : DTR et RTS a false, sinon on reset l ESP32 en ouvrant
            $sp.DtrEnable  = $false
            $sp.RtsEnable  = $false
            $sp.ReadTimeout  = 200
            $sp.WriteTimeout = 500
            $sp.NewLine = "`n"
            $sp.Open()
            Write-Log "--- connecte sur $p a $Baud bauds ---"
            $lastPortMsg = $p
            $buffer = ""
        }
        catch {
            if ($lastPortMsg -ne "fail-$p") {
                Write-Log "--- $p indisponible ($($_.Exception.Message)) ---"
                $lastPortMsg = "fail-$p"
            }
            $sp = $null
            Start-Sleep -Milliseconds 1500
            continue
        }
    }

    # ---------------------------------------------------- lecture
    try {
        $chunk = $sp.ReadExisting()
        if ($chunk -and $chunk.Length -gt 0) {
            $buffer += $chunk
            while ($buffer.Contains("`n")) {
                $i = $buffer.IndexOf("`n")
                $line = $buffer.Substring(0, $i).TrimEnd("`r")
                $buffer = $buffer.Substring($i + 1)
                if ($line.Length -gt 0) { Write-Log $line }
            }
        }
    }
    catch {
        Write-Log "--- lecture interrompue : $($_.Exception.Message) ---"
        try { $sp.Close() } catch {}
        $sp = $null
        Start-Sleep -Milliseconds 1000
        continue
    }

    # ---------------------------------------------------- commandes a envoyer
    if (Test-Path $cmdPath) {
        try {
            $lines = Get-Content $cmdPath -ErrorAction Stop
            Remove-Item $cmdPath -Force -ErrorAction SilentlyContinue
            foreach ($l in $lines) {
                $t = $l.Trim()
                if ($t.Length -gt 0) {
                    $sp.Write($t + "`n")
                    Write-Log ">>> ENVOYE : $t"
                    Start-Sleep -Milliseconds 60
                }
            }
        }
        catch {
            Write-Log "--- envoi impossible : $($_.Exception.Message) ---"
        }
    }

    # ---------------------------------------------------- rotation du log
    if ((Get-Item $logPath -ErrorAction SilentlyContinue).Length -gt 20MB) {
        Move-Item $logPath "$logPath.old" -Force
        Write-Log "=== log pivote ==="
    }

    Start-Sleep -Milliseconds 30
}

if ($sp -and $sp.IsOpen) { $sp.Close() }
Write-Log "=== moniteur arrete ==="
