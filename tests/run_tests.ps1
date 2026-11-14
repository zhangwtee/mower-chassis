# 在 PC 上编译并跑单元测试，不需要硬件。
#
# 用法：
#   powershell -ExecutionPolicy Bypass -File tests\run_tests.ps1
#
# 需要 gcc。先看 PATH 里有没有；没有就试几个常见位置（Qt 自带的 MinGW 等）。
# 都找不到就报错退出，别静默跳过——没有编译器却报"全过"是最坏的情况。

$ErrorActionPreference = 'Stop'

$candidates = @()

$onPath = Get-Command gcc.exe -ErrorAction SilentlyContinue
if ($onPath) { $candidates += $onPath.Source }

$candidates += @(
    'D:\Qt\Tools\mingw1310_64\bin\gcc.exe',
    'C:\Qt\Tools\mingw1310_64\bin\gcc.exe',
    'C:\msys64\mingw64\bin\gcc.exe',
    'C:\MinGW\bin\gcc.exe'
)

$gcc = $null
foreach ($c in $candidates) {
    if (Test-Path $c) { $gcc = $c; break }
}

if (-not $gcc) {
    Write-Error "找不到 gcc。装一个 MinGW，或者把 gcc 所在目录加进 PATH。"
    exit 2
}

# gcc 旁边的 DLL 得能被找到，否则 cc1.exe 会静默退出（0xC0000135）
$gccDir = Split-Path $gcc -Parent
$env:PATH = "$gccDir;$env:PATH"

$root  = Split-Path $PSScriptRoot -Parent
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$exe = Join-Path $build 'test.exe'

# 只编纯逻辑模块：它们不 include 任何 STM32 头
$sources = @(
    (Join-Path $PSScriptRoot 'test_main.c'),
    (Join-Path $root 'app\protocol.c'),
    (Join-Path $root 'app\telemetry.c'),
    (Join-Path $root 'app\kinematics.c'),
    (Join-Path $root 'app\pi.c'),
    (Join-Path $root 'app\chassis_safety.c'),
    (Join-Path $root 'app\blade.c'),
    (Join-Path $root 'app\chassis.c'),
    (Join-Path $root 'app\ring.c'),
    (Join-Path $root 'app\speed_smooth.c'),
    (Join-Path $root 'app\filter.c'),
    (Join-Path $root 'app\app_param.c'),
    (Join-Path $root 'app\avoid.c'),
    (Join-Path $root 'app\params.c'),
    (Join-Path $root 'app\click.c'),
    (Join-Path $root 'app\park.c'),
    (Join-Path $root 'app\joy_guard.c')
)

# -Werror：警告当错误。能用编译器挡住的，别留给上机
$gccArgs = @(
    '-std=c99', '-Wall', '-Wextra', '-Werror', '-O2',
    '-I', (Join-Path $root 'app'),
    '-I', (Join-Path $root 'bsp'),
    '-I', $PSScriptRoot,
    '-o', $exe
) + $sources

Write-Host "gcc: $gcc"
Write-Host "编译中..."
& $gcc @gccArgs
if ($LASTEXITCODE -ne 0) {
    Write-Host "编译失败" -ForegroundColor Red
    exit 1
}

Write-Host "运行测试..."
& $exe
$code = $LASTEXITCODE

if ($code -eq 0) {
    Write-Host "`n全部通过" -ForegroundColor Green
} else {
    Write-Host "`n有失败项" -ForegroundColor Red
}
exit $code
