# Set-ExecutionPolicy -ExecutionPolicy Bypass -Scope Process
# path: %localappdata%\Temp\arduino\sketches\<hash>\compile_commands.json
param (
    [string]$path = $( throw "Sketch path is required" )
)

# Convert to FileInfo
$srcPath = Get-Item -Path $path
$tmpDir = $srcPath.Directory
$sketchDir = Join-Path -Path $tmpDir -ChildPath "sketch"

$data = Get-Content -Raw -Path $srcPath | ConvertFrom-Json

function IsCustom
{
    param ([System.IO.DirectoryInfo]$Path)

    # Write-Output @("cmp", $Path.FullName, $tmpDir.FullName)

    if ($Path.FullName -eq $tmpDir.FullName)
    {
        return $true
    }

    if ($null -eq $Path.Parent)
    {
        return $false
    }

    return IsCustom -Path $Path.Parent
}

foreach ($obj in $data)
{
    $file = Get-Item -Path $obj.file
    if (!(IsCustom -Path $file.Directory))
    {
        continue
    }

    if ( $file.FullName.EndsWith(".ino.cpp"))
    {
        continue
    }

    $rep = $file.FullName.Replace($sketchDir, $PSScriptRoot)
    #Write-Output $file.FullName
    #Write-Output $rep

    $idx = [array]::IndexOf($obj.arguments, $obj.file)
    $obj.arguments[$idx] = $rep

    $obj.file = $rep
}

Write-Output $data | ConvertTo-Json | Out-File -FilePath compile_commands.json
