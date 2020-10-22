Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test1a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 4 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x4_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x4_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test2a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell-199lights.txt -size 256 256 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell-199lights.txt_256x256x1_Stage3.exe.bmp Outputs_REFERENCE\cornell-199lights.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test5a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/dudes.txt -size 256 256 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\dudes.txt_256x256x1_Stage3.exe.bmp Outputs_REFERENCE\dudes.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test7a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test1b.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 4 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x4_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x4_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test2b.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell-199lights.txt -size 256 256 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell-199lights.txt_256x256x1_Stage3.exe.bmp Outputs_REFERENCE\cornell-199lights.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test5b.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/dudes.txt -size 256 256 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\dudes.txt_256x256x1_Stage3.exe.bmp Outputs_REFERENCE\dudes.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage3_tests1_test7b.bmp
cmd \k