# Activate the SO-101 environment in the current PowerShell session.
#   . .\activate.ps1
& "$env:USERPROFILE\anaconda3\shell\condabin\conda-hook.ps1"
conda activate "$PSScriptRoot\.venv"
