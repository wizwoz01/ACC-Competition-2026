$ErrorActionPreference = "Stop"

$Base = Split-Path -Parent $MyInvocation.MyCommand.Path
$Runner = Join-Path $Base "qcar2_detailed_scenario_runner.py"
$Setup = Join-Path $Base "Setup_Real_Scenario.py"
$Model = Join-Path $Base "best.torchscript"
$Labels = Join-Path $Base "sign_labels.txt"
$Waypoints = Join-Path $Base "waypoints.txt"

Push-Location $Base

$setupProc = $null
try {
	if (Test-Path $Setup) {
		Write-Host "[LAUNCH] Starting 1/10-scale setup in background..."
		$setupProc = Start-Process -FilePath "python" -ArgumentList @($Setup) -WorkingDirectory $Base -PassThru
		Start-Sleep -Seconds 2
	}

	python $Runner --waypoints $Waypoints --speed 0.06 --sign-model $Model --sign-labels $Labels --sign-device cpu --no-recover
}
finally {
	if ($null -ne $setupProc) {
		try {
			if (-not $setupProc.HasExited) {
				Write-Host "[LAUNCH] Stopping setup process (PID $($setupProc.Id))..."
				Stop-Process -Id $setupProc.Id -Force
			}
		}
		catch {
			Write-Warning "Failed to stop setup process: $($_.Exception.Message)"
		}
	}

	Pop-Location
}
