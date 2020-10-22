@ECHO OFF
set runs=%1
if "%1"=="" set runs=1
@ECHO ON
Release\RayTracerAss1.exe -runs %runs% -input Scenes/cornell.txt -size 1024 1024 -samples 1
Release\RayTracerAss1.exe -runs %runs% -input Scenes/cornell.txt -size 1024 1024 -samples 4
Release\RayTracerAss1.exe -runs %runs% -input Scenes/cornell.txt -size 500 300 -samples 1
Release\RayTracerAss1.exe -runs %runs% -input Scenes/allmaterials.txt -size 1000 1000 -samples 1
Release\RayTracerAss1.exe -runs %runs% -input Scenes/cornell-199lights.txt -size 256 256 -samples 1
Release\RayTracerAss1.exe -runs %runs% -input Scenes/5000spheres.txt -size 480 270 -samples 1
Release\RayTracerAss1.exe -runs %runs% -input Scenes/dudes.txt -size 256 256 -samples 1
cmd \k
