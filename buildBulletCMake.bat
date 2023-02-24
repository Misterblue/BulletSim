rem Script to build Bullet on a target system.

set MACH=x64
set BULLETDIR=bullet3

set BUILDDIR=bullet-build

cd %BULLETDIR%
mkdir -p %BUILDDIR%
cd %BUILDDIR%

echo "=== Building Bullet in dir %BULLETDIR% for arch %MACH% into %BUILDDIR%"

cmake -G "Visual Studio 17 2022" -A %MACH% -DDOTNET_SDK=ON -DBUILD_BULLET3=ON -DBUILD_EXTRAS=ON -DBUILD_INVERSE_DYNAMIC_EXTRA=OFF -DBUILD_BULLET_ROBOTICS_GUI_EXTRA=OFF -DBUILD_BULLET_ROBOTICS_EXTRA=OFF -DBUILD_OBJ2SDF_EXTRA=OFF -DBUILD_SERIALIZE_EXTRA=OFF -DBUILD_CONVEX_DECOMPOSITION_EXTRA=ON -DBUILD_HACD_EXTRA=ON -DBUILD_GIMPACTUTILS_EXTRA=OFF -DBUILD_CPU_DEMOS=OFF -DBUILD_BULLET2_DEMOS=OFF -DBUILD_ENET=OFF -DBUILD_PYBULLET=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -DINSTALL_EXTRA_LIBS=ON -DINSTALL_LIBS=ON -DCMAKE_CXX_FLAGS="-fPIC" -DCMAKE_BUILD_TYPE=Release ..

msbuild -p:Configuration=Release BULLET_PHYSICS.sln

rem Copy the .lib files into the target lib directory
Set-Location ..\..
New-Item -ItemType Directory -Path lib -Force
Write-Host "=== Copy .lib files into the lib dir"
Get-Children -Path bullet3\bullet-build\* -Include *.lib -Recurse | ForEach-Object {
    Copy-Item $_.FullName -Destination lib -Force
}

rem Copy the .h files into the target include directory
Get-ChildItem -Path bullet3\src\* -Include *.h -Recurse | ForEach-Object {
    $xxxx = $_.Fullname -replace ".*\bullet3\src\","include\"
    New-Item -ItemType Directory -Path $xxxx -Force
    Copy-Item $_.FullName -Destination $xxxx -Force
}

rem Copy the .h files from Extras into the target include directory
Get-ChildItem -Path bullet3\Extras\* -Include *.h -Recurse | ForEach-Object {
    $xxxx = $_.Fullname -replace ".*\bullet3\Extras\","include\"
    New-Item -ItemType Directory -Path $xxxx -Force
    Copy-Item $_.FullName -Destination $xxxx -Force
}

