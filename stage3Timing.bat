@ECHO OFF
set runs=%1
if "%1"=="" set runs=3
set threads=%2
if "%2"=="" set threads=8
set blocksize=%3
if "%3"=="" set blocksize=64
@ECHO ON
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/cornell.txt -size 1024 1024 -samples 1
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/cornell.txt -size 1024 1024 -samples 4
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/cornell.txt -size 500 300 -samples 1
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/allmaterials.txt -size 1000 1000 -samples 1
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/cornell-199lights.txt -size 256 256 -samples 1
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/5000spheres.txt -size 480 270 -samples 1
Release\Stage3.exe -runs %runs% -threads %threads% -blockSize %blocksize% -input Scenes/dudes.txt -size 256 256 -samples 1
cmd \k