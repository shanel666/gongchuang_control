# 编译并运行 chassis.c 的 PC 端回归测试（4 种配置）
# 用法：在工程根目录执行
#   powershell -NoProfile -ExecutionPolicy Bypass -File tmp_chassis_test/run_tests.ps1
$ErrorActionPreference = 'Stop'

# 让 test_chassis.exe 输出的中文（UTF-8）在控制台正常显示
try { [Console]::OutputEncoding = [System.Text.Encoding]::UTF8 } catch { }

$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$root = Split-Path -Parent $here
Push-Location $root

try {
    $variants = @(
        @{ name = '8B-sync-off'; args = @() },
        @{ name = '9B-sync-off'; args = @('-DCHASSIS_FRAME_LAYOUT=CHASSIS_FRAME_LAYOUT_9B') },
        @{ name = '8B-sync-on';  args = @('-DCHASSIS_USE_SYNC=1') },
        @{ name = '9B-sync-on';  args = @('-DCHASSIS_FRAME_LAYOUT=CHASSIS_FRAME_LAYOUT_9B', '-DCHASSIS_USE_SYNC=1') }
    )

    $failed = $false

    foreach ($v in $variants) {
        $exe = Join-Path $here ('build_' + $v.name + '.exe')
        $args = @(
            '-std=c99', '-Wall', '-Wextra',
            '-I', (Join-Path $here 'stub'),
            '-I', (Join-Path $root 'app'),
            (Join-Path $here 'test_chassis.c'),
            (Join-Path $root 'app/chassis.c'),
            '-lm', '-o', $exe
        ) + $v.args

        & gcc @args
        if ($LASTEXITCODE -ne 0) {
            Write-Host "[$($v.name)] BUILD FAIL"
            $failed = $true
            continue
        }

        $output = & $exe 2>&1
        $result = ($output | Select-String -Pattern 'ALL PASS|FAIL' | ForEach-Object { $_.Line.Trim() })
        Write-Host "[$($v.name)] $result"
        if ($LASTEXITCODE -ne 0) {
            $failed = $true
            $output | Select-String -Pattern 'FAIL' | ForEach-Object { Write-Host ('  ' + $_.Line.Trim()) }
        }
    }

    if ($failed) {
        Write-Host 'SOME VARIANTS FAILED'
        exit 1
    }
    Write-Host 'ALL VARIANTS PASS'
}
finally {
    Pop-Location
}
