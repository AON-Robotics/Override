param([string[]] $Test = @())

$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $env:TEMP 'override-aon-host-tests'
$vcVars = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat'

if (-not (Test-Path -LiteralPath $vcVars)) {
    throw "Visual Studio C++ environment was not found at $vcVars"
}

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

if ($Test.Count -eq 0 -or 'static-path-generator-test' -in $Test) {
    & python (Join-Path $repositoryRoot 'tests/static-path-generator-test.py')
    if ($LASTEXITCODE -ne 0) { throw 'Static path converter tests failed' }
}
& python (Join-Path $repositoryRoot 'tools/generate-static-path.py') `
    (Join-Path $repositoryRoot 'static/path.jerryio.txt') `
    (Join-Path $repositoryRoot 'include/aon/generated/static-path.hpp')
if ($LASTEXITCODE -ne 0) { throw 'Static path generation failed' }

function Invoke-CppTest {
    param(
        [Parameter(Mandatory)] [string] $Name,
        [Parameter(Mandatory)] [string[]] $Sources
    )

    if ($Test.Count -gt 0 -and $Name -notin $Test) { return }

    $executable = Join-Path $buildDirectory "$Name.exe"
    $quotedSources = $Sources | ForEach-Object {
        '"' + (Join-Path $repositoryRoot $_) + '"'
    }
    $includeDirectory = Join-Path $repositoryRoot 'include'
    $compile = 'cl.exe /nologo /std:c++17 /EHsc /W4 /WX ' +
        "/I`"$includeDirectory`" " +
        ($quotedSources -join ' ') +
        " /Fe:`"$executable`""
    $command = "call `"$vcVars`" >nul && $compile"

    & $env:ComSpec /d /s /c $command
    if ($LASTEXITCODE -ne 0) {
        throw "$Name compilation failed with exit code $LASTEXITCODE"
    }

    & $executable
    if ($LASTEXITCODE -ne 0) {
        throw "$Name failed with exit code $LASTEXITCODE"
    }
}

Invoke-CppTest -Name 'auton-selection-test' -Sources @(
    'tests/auton-selection-test.cpp'
)

Invoke-CppTest -Name 'odometry-test' -Sources @(
    'tests/odometry-test.cpp'
)

Invoke-CppTest -Name 'native-follower-test' -Sources @(
    'tests/native-follower-test.cpp'
)

Invoke-CppTest -Name 'follow-runtime-test' -Sources @(
    'tests/follow-runtime-test.cpp'
)
