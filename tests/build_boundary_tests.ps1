$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$vcRoot = 'D:/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.42.34433'
$ucrtInclude = 'C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/ucrt'
Push-Location $repoRoot
try {
    & "$vcRoot/bin/Hostx64/x64/cl.exe" /nologo /c /GS- /Od /utf-8 /wd4828 /Itests/stubs /Ihardware/Emm_v5 "/I$vcRoot/include" "/I$ucrtInclude" tests/boundary_harness.c /Fotests/boundary_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'Host C compilation failed' }
    & "$vcRoot/bin/Hostx64/x64/link.exe" /nologo /dll /noentry /nodefaultlib /out:tests/boundary_harness.dll tests/boundary_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'Host DLL linking failed' }
} finally {
    Pop-Location
}
