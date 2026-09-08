param([string[]] $Test = @())

$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent $PSScriptRoot
$buildDirectory = Join-Path $env:TEMP 'override-pathjerry-host-tests'
$vcVars = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat'

if (-not (Test-Path -LiteralPath $vcVars)) {
    throw "Visual Studio C++ environment was not found at $vcVars"
}

New-Item -ItemType Directory -Force -Path $buildDirectory | Out-Null

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

Invoke-CppTest -Name 'path-jerryio-test' -Sources @(
    'tests/path-jerryio-test.cpp',
    'src/aon/jerryio/path-jerryio.cpp'
)

Invoke-CppTest -Name 'path-follower-test' -Sources @(
    'tests/path-follower-test.cpp',
    'src/aon/jerryio/path-follower.cpp'
)

Invoke-CppTest -Name 'path-execution-policy-test' -Sources @(
    'tests/path-execution-policy-test.cpp',
    'src/aon/jerryio/path-following.cpp'
)

Invoke-CppTest -Name 'path-actions-test' -Sources @(
    'tests/path-actions-test.cpp',
    'src/aon/jerryio/path-actions.cpp'
)

Invoke-CppTest -Name 'path-telemetry-test' -Sources @(
    'tests/path-telemetry-test.cpp',
    'src/aon/jerryio/path-telemetry.cpp'
)

Invoke-CppTest -Name 'path-transform-test' -Sources @(
    'tests/path-transform-test.cpp',
    'src/aon/jerryio/path-transform.cpp'
)

Invoke-CppTest -Name 'routine-actions-test' -Sources @(
    'tests/routine-actions-test.cpp',
    'src/aon/jerryio/routine-actions.cpp'
)

Invoke-CppTest -Name 'auton-selection-test' -Sources @(
    'tests/auton-selection-test.cpp'
)

Invoke-CppTest -Name 'basic-uturn-test' -Sources @(
    'tests/basic-uturn-test.cpp'
)

Invoke-CppTest -Name 'odometry-test' -Sources @(
    'tests/odometry-test.cpp',
    'src/aon/jerryio/path-jerryio.cpp',
    'src/aon/jerryio/path-transform.cpp',
    'src/aon/jerryio/path-follower.cpp'
)

Invoke-CppTest -Name 'path-jerryio-asset-test' -Sources @(
    'tests/path-jerryio-asset-test.cpp',
    'src/aon/jerryio/path-jerryio.cpp',
    'src/aon/jerryio/path-follower.cpp',
    'src/aon/jerryio/path-transform.cpp',
    'src/aon/jerryio/path-actions.cpp'
)
