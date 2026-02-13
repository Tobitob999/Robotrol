<#
Codex Runner - safe command gateway for robocontrol_v6_0
Usage: .\codex.ps1 <command> [args...]

Commands:
  rg ...
  git ...
  python <script.py> [args...]
  python -m <pytest|pip|venv|ruff|black|mypy> [args...]
  pytest [args...]
  pip [args...]
  ruff [args...]
  black [args...]
  mypy [args...]
  uv [args...]
  pyinstaller [args...]
  pre-commit [args...]
  start latest
  help
#>

[CmdletBinding()]
param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]]$Args
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path

function Show-Help {
  Write-Host "Codex Runner - safe command gateway for this repo."
  Write-Host "Usage: .\codex.ps1 <command> [args...]"
  Write-Host ""
  Write-Host "Commands:"
  Write-Host "  rg <args...>"
  Write-Host "  git <status|diff|log|show|branch|rev-parse|ls-files|describe|remote|fetch|add|commit|checkout|switch|restore|pull|push|merge|stash|tag> <args...>"
  Write-Host "  python <script.py> [args...]"
  Write-Host "  python -m <pytest|pip|venv|ruff|black|mypy> [args...]"
  Write-Host "  pytest [args...]"
  Write-Host "  pip [args...]"
  Write-Host "  ruff [args...]"
  Write-Host "  black [args...]"
  Write-Host "  mypy [args...]"
  Write-Host "  uv [args...]"
  Write-Host "  pyinstaller [args...]"
  Write-Host "  pre-commit [args...]"
  Write-Host "  start latest"
  Write-Host "  help"
}

function Resolve-RepoPath {
  param([string]$Path)
  if ([System.IO.Path]::IsPathRooted($Path)) {
    throw "Absolute paths are not allowed: $Path"
  }
  if ($Path -match "\\.\\.") {
    throw "Parent paths are not allowed: $Path"
  }
  $full = [System.IO.Path]::GetFullPath((Join-Path $RepoRoot $Path))
  if (-not $full.StartsWith($RepoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Path escapes repo root: $Path"
  }
  return $full
}

function Invoke-InRepo {
  param([string]$Exe, [string[]]$ExeArgs)
  Push-Location $RepoRoot
  try {
    & $Exe @ExeArgs
    return $LASTEXITCODE
  } finally {
    Pop-Location
  }
}

if (-not $Args -or $Args.Count -eq 0) {
  Show-Help
  exit 2
}

$cmd = $Args[0].ToLower()
$rest = if ($Args.Count -gt 1) { $Args[1..($Args.Count - 1)] } else { @() }

switch ($cmd) {
  "help" { Show-Help; exit 0 }

  "rg" {
    $code = Invoke-InRepo "rg" $rest
    exit $code
  }

  "git" {
    if ($rest.Count -eq 0) { throw "git requires a subcommand" }
    $sub = $rest[0].ToLower()
    $allowed = @(
      "status","diff","log","show","branch","rev-parse","ls-files","describe","remote","fetch",
      "add","commit","checkout","switch","restore","pull","push","merge","stash","tag"
    )
    if ($allowed -notcontains $sub) {
      throw ("git subcommand not allowed: " + $sub + ". Allowed: " + ($allowed -join ", "))
    }
    $code = Invoke-InRepo "git" $rest
    exit $code
  }

  "python" {
    if ($rest.Count -eq 0) { throw "python requires a script or -m <module>" }
    if ($rest[0] -eq "-m") {
      if ($rest.Count -lt 2) { throw "python -m requires a module" }
      $mod = $rest[1].ToLower()
      $allowed = @("pytest","pip","venv","ruff","black","mypy")
      if ($allowed -notcontains $mod) {
        throw ("python -m only allows: " + ($allowed -join ", "))
      }
      $code = Invoke-InRepo "python" $rest
      exit $code
    }

    $script = Resolve-RepoPath $rest[0]
    if (-not $script.ToLower().EndsWith(".py")) {
      throw "Only .py scripts are allowed"
    }
    $args2 = @($script)
    if ($rest.Count -gt 1) { $args2 += $rest[1..($rest.Count - 1)] }
    $code = Invoke-InRepo "python" $args2
    exit $code
  }

  "pytest" {
    $code = Invoke-InRepo "python" (@("-m","pytest") + $rest)
    exit $code
  }

  "pip" {
    $code = Invoke-InRepo "python" (@("-m","pip") + $rest)
    exit $code
  }

  "ruff" {
    $code = Invoke-InRepo "ruff" $rest
    exit $code
  }

  "black" {
    $code = Invoke-InRepo "black" $rest
    exit $code
  }

  "mypy" {
    $code = Invoke-InRepo "mypy" $rest
    exit $code
  }

  "uv" {
    $code = Invoke-InRepo "uv" $rest
    exit $code
  }

  "pyinstaller" {
    $code = Invoke-InRepo "pyinstaller" $rest
    exit $code
  }

  "pre-commit" {
    $code = Invoke-InRepo "pre-commit" $rest
    exit $code
  }

  "start" {
    if ($rest.Count -eq 0) { throw "start requires a target" }
    $target = $rest[0].ToLower()
    if ($target -ne "latest") {
      throw "start only supports 'latest'"
    }
    $pattern = "Robotrol_FluidNC_v6_*.py"
    $latest = Get-ChildItem -Path $RepoRoot -Filter $pattern | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if (-not $latest) {
      throw "No file matching $pattern found in repo root"
    }
    $code = Invoke-InRepo "python" @($latest.Name)
    exit $code
  }

  default {
    throw "Unknown command: $cmd. Use: .\\codex.ps1 help"
  }
}
