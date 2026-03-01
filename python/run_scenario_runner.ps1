#$ErrorActionPreference = "Stop"

#$Base = Split-Path -Parent $MyInvocation.MyCommand.Path
#$Runner = Join-Path $Base "qcar2_detailed_scenario_runner.py"
#$Model = Join-Path $Base "best.torchscript"
#$Labels = Join-Path $Base "sign_labels.txt"
#$Waypoints = Join-Path $Base "waypoints.txt"

#python $Runner --waypoints $Waypoints --sign-model $Model --sign-labels $Labels --sign-device cpu

python qcar2_detailed_scenario_runner.py --waypoints waypoints.txt --sign-model best.torchscript --sign-labels sign_labels.txt --lane-model path/to/lane_model.torchscript --lane-device cuda

