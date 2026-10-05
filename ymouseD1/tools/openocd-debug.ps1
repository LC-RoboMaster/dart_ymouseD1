<#
.SYNOPSIS
    用 OpenOCD + arm-none-eabi-gdb 一键调试 ymouseD1。

.NOTES
    【编码】本文件必须保存为 UTF-8 **带 BOM**。Windows PowerShell 5.1 在没有 BOM
    时会按 GBK 解析 .ps1,文件里的中文会变乱码并报语法错误。用 VS Code 保存时注意
    右下角编码要显示 "UTF-8 with BOM"。

.DESCRIPTION
    1. 可选地先编译(build/Debug/ymouseD1.elf)
    2. 后台启动 openocd -f tools/openocd.cfg
    3. 等 GDB server(3333)起来后,用 tools/debug.gdb 启动 GDB 并把固件烧进去
    4. GDB 退出后自动关掉 openocd

.EXAMPLE
    pwsh -File tools/openocd-debug.ps1
    只连不烧录(固件已在片内):
    pwsh -File tools/openocd-debug.ps1 -SkipBuild -NoLoad
#>
[CmdletBinding()]
param(
    # 固件路径,默认 build/Debug/ymouseD1.elf
    [string]$Elf = '',
    # 跳过编译,直接用现有 elf
    [switch]$SkipBuild,
    # 不执行 debug.gdb 里的 load(只 attach 现有固件,适合只想看现场)
    [switch]$NoLoad
)

$ErrorActionPreference = 'Stop'

$toolsDir = $PSScriptRoot
$projDir  = Split-Path -Parent $toolsDir
$cfgFile  = Join-Path $toolsDir 'openocd.cfg'
$gdbFile  = Join-Path $toolsDir 'debug.gdb'

if (-not $Elf) { $Elf = Join-Path $projDir 'build\Debug\ymouseD1.elf' }
if (-not (Test-Path $Elf)) { throw "找不到固件: $Elf(先去掉 -SkipBuild 编译一次)" }

# ---------------------------- 找工具 ----------------------------
$openocd = (Get-Command openocd -ErrorAction SilentlyContinue).Source
if (-not $openocd) { throw '没找到 openocd,请确认它在 PATH 里' }

$gdb = (Get-Command arm-none-eabi-gdb -ErrorAction SilentlyContinue).Source
if (-not $gdb) {
    $cand = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe'
    if (Test-Path $cand) { $gdb = $cand } else { throw '没找到 arm-none-eabi-gdb' }
}

Write-Host "openocd : $openocd"
Write-Host "gdb     : $gdb"
Write-Host "elf     : $Elf"

# ---------------------------- 编译 ----------------------------
if (-not $SkipBuild) {
    Write-Host "`n=== 编译 ===" -ForegroundColor Cyan
    & cmake --build --preset Debug
    if ($LASTEXITCODE -ne 0) { throw "编译失败(exit $LASTEXITCODE)" }
}

# ---------------------------- 启动 OpenOCD ----------------------------
$log = Join-Path $env:TEMP 'openocd-ymouseD1.log'
Remove-Item $log -ErrorAction SilentlyContinue

Write-Host "`n=== 启动 OpenOCD ===" -ForegroundColor Cyan
$ocd = Start-Process -FilePath $openocd `
    -ArgumentList @('-f', $cfgFile) `
    -PassThru -NoNewWindow `
    -RedirectStandardOutput "$log.out" -RedirectStandardError $log

function Test-GdbPort([int]$Port) {
    try {
        $c = New-Object System.Net.Sockets.TcpClient
        $c.Connect('127.0.0.1', $Port)
        $c.Close()
        return $true
    } catch { return $false }
}

$ok = $false
for ($i = 0; $i -lt 60; $i++) {
    if ($ocd.HasExited) {
        $ocd.WaitForExit()   # 等句柄回收,否则 ExitCode 取不到
        Write-Host (Get-Content $log -Raw -ErrorAction SilentlyContinue)
        Write-Host (Get-Content "$log.out" -Raw -ErrorAction SilentlyContinue)
        throw "OpenOCD 启动失败(exit $($ocd.ExitCode)),日志见 $log"
    }
    if (Test-GdbPort 3333) { $ok = $true; break }
    Start-Sleep -Milliseconds 250
}
if (-not $ok) { Stop-Process -Id $ocd.Id -Force -ErrorAction SilentlyContinue; throw '等 GDB server(3333)超时' }
Write-Host "OpenOCD 已就绪(GDB server 127.0.0.1:3333),日志: $log"

# ---------------------------- 启动 GDB ----------------------------
try {
    $gdbArgs = @($Elf)
    if (-not $NoLoad) { $gdbArgs += @('-x', $gdbFile) }
    else { $gdbArgs += @('-ex', 'target extended-remote localhost:3333') }

    Write-Host "`n=== 启动 GDB ===" -ForegroundColor Cyan
    & $gdb @gdbArgs
}
finally {
    if ($ocd -and -not $ocd.HasExited) {
        Stop-Process -Id $ocd.Id -Force -ErrorAction SilentlyContinue
        Write-Host "`n已关闭 OpenOCD" -ForegroundColor DarkGray
    }
}
