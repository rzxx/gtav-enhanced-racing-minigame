param(
    [string]$GtaDir = "D:\SteamLibrary\steamapps\common\Grand Theft Auto V Enhanced",
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

dotnet build "$root\src\StreetRacing\StreetRacing.csproj" -c $Configuration -p:GtaEnhancedDir="$GtaDir"
$dll = "$root\src\StreetRacing\bin\$Configuration\net48\StreetRacing.dll"
if (!(Test-Path $dll)) { throw "Build output not found: $dll" }

$scripts = Join-Path $GtaDir "scripts"
if (!(Test-Path $scripts)) { throw "scripts folder not found: $scripts" }
Copy-Item $dll (Join-Path $scripts "StreetRacing.dll") -Force

$ini = Join-Path $scripts "StreetRacing.ini"
if (!(Test-Path $ini)) {
    Copy-Item "$root\config\StreetRacing.ini" $ini
    Write-Output "Installed default StreetRacing.ini"
}
Write-Output "Installed StreetRacing.dll -> $scripts (press Insert in game to reload scripts)"
