$ErrorActionPreference = 'Stop'

$repositoryRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repositoryRoot
try {
    $testProjects = @(
        'tests/SpiderLocomotion.Tests/SpiderLocomotion.Tests.csproj',
        'tests/SpiderLocomotion.Integration/SpiderLocomotion.Integration.csproj',
        'tests/SpiderNativeResources.Tests/SpiderNativeResources.Tests.csproj',
        'tests/SpiderTargetSurface.Tests/SpiderTargetSurface.Tests.csproj',
        'tests/SpiderNavigation.Tests/SpiderNavigation.Tests.csproj',
        'tests/SpiderAi.Integration/SpiderAi.Integration.csproj'
    )

    $discoveredProjects = @(Get-ChildItem 'tests' -Recurse -Filter '*.csproj' |
        ForEach-Object { $_.FullName.Substring($repositoryRoot.Length + 1).Replace('\', '/') } |
        Sort-Object)
    $expectedProjects = @($testProjects | Sort-Object)
    $missing = @($discoveredProjects | Where-Object { $_ -notin $expectedProjects })
    $stale = @($expectedProjects | Where-Object { $_ -notin $discoveredProjects })
    if ($missing.Count -gt 0 -or $stale.Count -gt 0) {
        throw "Test project inventory mismatch. Unregistered: $($missing -join ', '). Missing: $($stale -join ', '). Update tests/run-all.ps1."
    }

    foreach ($project in $testProjects) {
        Write-Host "`n=== Build $project ===" -ForegroundColor Cyan
        dotnet build $project --configuration Release --verbosity minimal
        if ($LASTEXITCODE -ne 0) { throw "Build failed: $project (exit $LASTEXITCODE)" }
    }

    $runs = @(
        @{ Project = $testProjects[0]; Arguments = @() },
        @{ Project = $testProjects[1]; Arguments = @('--case', 'all') },
        @{ Project = $testProjects[2]; Arguments = @() },
        @{ Project = $testProjects[3]; Arguments = @() },
        @{ Project = $testProjects[4]; Arguments = @() },
        @{ Project = $testProjects[5]; Arguments = @('--case', 'all') }
    )

    foreach ($run in $runs) {
        Write-Host "`n=== Test $($run.Project) ===" -ForegroundColor Cyan
        dotnet run --project $run.Project --configuration Release --no-build -- @($run.Arguments)
        if ($LASTEXITCODE -ne 0) { throw "Tests failed: $($run.Project) (exit $LASTEXITCODE)" }
    }

    Write-Host "`nAll $($runs.Count) test harnesses passed." -ForegroundColor Green
}
finally {
    Pop-Location
}
