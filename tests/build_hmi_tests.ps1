$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$vcRoot = 'D:/Microsoft Visual Studio/2022/Community/VC/Tools/MSVC/14.42.34433'
$ucrtInclude = 'C:/Program Files (x86)/Windows Kits/10/Include/10.0.22621.0/ucrt'
Push-Location $repoRoot
try {
    & "$vcRoot/bin/Hostx64/x64/cl.exe" /nologo /c /GS- /Od /utf-8 /Itests/hmi_stubs "/I$vcRoot/include" "/I$ucrtInclude" tests/hmi_harness.c /Fotests/hmi_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'HMI host C compilation failed' }
    & "$vcRoot/bin/Hostx64/x64/link.exe" /nologo /dll /noentry /nodefaultlib /out:tests/hmi_harness.dll tests/hmi_harness.obj
    if ($LASTEXITCODE -ne 0) { throw 'HMI host DLL linking failed' }
} finally { Pop-Location }
