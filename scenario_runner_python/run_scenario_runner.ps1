$ErrorActionPreference = "Stop"

$Base = Split-Path -Parent $MyInvocation.MyCommand.Path
$Runner = Join-Path $Base "qcar2_detailed_scenario_runner.py"
$Model = Join-Path $Base "best.torchscript"
$Labels = Join-Path $Base "sign_labels.txt"
$Waypoints = Join-Path $Base "waypoints.txt"

python $Runner --waypoints $Waypoints --sign-model $Model --sign-labels $Labels --sign-device cpu
