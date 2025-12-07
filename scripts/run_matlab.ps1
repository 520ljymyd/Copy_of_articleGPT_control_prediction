<#+
run_matlab.ps1

Simple PowerShell wrapper to call send_to_matlab.py with the provided .m file.

Usage:
  .\run_matlab.ps1 <path-to-m-file> [-Name <engineName>]
#>
param(
    [Parameter(Mandatory=$true, Position=0)]
    [string]$MFile,
    [string]$Name = 'vscode'
)

# Resolve paths
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$repoRoot = Resolve-Path "$scriptDir\.."
$py = 'python'  # assume python is on PATH; change if needed

if (-not (Test-Path $MFile)) {
    Write-Error "File not found: $MFile"
    exit 2
}

$mabs = (Resolve-Path $MFile).Path
Write-Host "Running $mabs in shared MATLAB (engine name: $Name) ..."
Start-Process -NoNewWindow -Wait -FilePath $py -ArgumentList ("`"$($repoRoot.Path)\send_to_matlab.py`"", "`"$mabs`"", "--name", "$Name")
