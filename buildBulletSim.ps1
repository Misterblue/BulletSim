
$BULLETVERSION = Get-Content -Path .\lib\VERSION
$BULLETSIMVERSION = Get-Content -Path .\VERSION

msbuild -p:BULLETVERSION=$BULLETVERSION -p:BULLETSIMVERSION=$BULLETSIMVERSION -p:Configuration=Release BulletSim.sln
