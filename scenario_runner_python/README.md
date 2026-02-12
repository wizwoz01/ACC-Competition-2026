# Scenario Runner 

## Included files
- `qcar2_detailed_scenario_runner.py`
- `waypoints.txt`
- `sign_labels.txt`
- `best.torchscript`
- `run_scenario_runner.ps1`
- `requirements.txt`

## Prerequisites
1. Python 3.9+ installed.
2. Quanser QCar / PAL Python SDK available in your Python environment (provides `pal.products.qcar` and `pal.utilities.vision`).
3. QLabs + required QCar services running.

## Setup
```powershell
$env:PYTHONPATH = "C:\<insert-where-quanser-github-is-located>\Quanser_Academic_Resources\0_libraries\python;$env:PYTHONPATH"
pip install -r requirements.txt
```


## Run
From this folder:

```powershell
.\run_scenario_runner.ps1
```

Or directly:

```powershell
python .\qcar2_detailed_scenario_runner.py --waypoints .\waypoints.txt --sign-model .\best.torchscript --sign-labels .\sign_labels.txt --sign-device cpu
```
