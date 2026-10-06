$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$vcRoot = 'D:/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.42.34433'
$ucrtInclude = 'C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/ucrt'
Push-Location $repoRoot
try {
    $canSource = Get-Content -LiteralPath Core/Src/can.c -Raw
    $callback = [regex]::Match($canSource, '(?s)void HAL_CAN_ErrorCallback\(CAN_HandleTypeDef \*hcan\)\s*\{.*?\n\}')
    if (!$callback.Success) { throw 'Cannot extract production CAN error callback' }
    [IO.File]::WriteAllText((Join-Path $repoRoot 'tests/can_error_callback.inc'), $callback.Value)
    & "$vcRoot/bin/Hostx64/x64/cl.exe" /nologo /c /GS- /Od /utf-8 /wd4828 /Itests/can_stubs /Ihardware/Emm_v5 "/I$vcRoot/include" "/I$ucrtInclude" tests/can_transport_harness.c /Fotests/can_transport_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'CAN host C compilation failed' }
    & "$vcRoot/bin/Hostx64/x64/link.exe" /nologo /dll /noentry /nodefaultlib /out:tests/can_transport_harness.dll tests/can_transport_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'CAN host DLL linking failed' }
} finally { Pop-Location }
