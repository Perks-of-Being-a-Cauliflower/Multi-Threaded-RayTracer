Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 500 300 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_500x300x1_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_500x300x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test3a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/allmaterials.txt -size 1000 1000 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\allmaterials.txt_1000x1000x1_Stage3.exe.bmp Outputs_REFERENCE\allmaterials.txt_1000x1000x1_RayTracerAss1.exe.bmp Outputs\stage3_tests2_test4a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/5000spheres.txt -size 480 270 -samples 1 -blockSize 8
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\5000spheres.txt_480x270x1_Stage3.exe.bmp Outputs_REFERENCE\5000spheres.txt_480x270x1_RayTracerAss1.exe.bmp Outputs\stage3_tests2_test6a.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 500 300 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_500x300x1_Stage3.exe.bmp Outputs_REFERENCE\cornell.txt_500x300x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test3b.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/allmaterials.txt -size 1000 1000 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\allmaterials.txt_1000x1000x1_Stage3.exe.bmp Outputs_REFERENCE\allmaterials.txt_1000x1000x1_RayTracerAss1.exe.bmp Outputs\stage3_tests2_test4b.bmp
Release\Stage3.exe -runs 1 -threads 8 -input Scenes/5000spheres.txt -size 480 270 -samples 1 -blockSize 64
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\5000spheres.txt_480x270x1_Stage3.exe.bmp Outputs_REFERENCE\5000spheres.txt_480x270x1_RayTracerAss1.exe.bmp Outputs\stage3_tests2_test6b.bmp
cmd \k
