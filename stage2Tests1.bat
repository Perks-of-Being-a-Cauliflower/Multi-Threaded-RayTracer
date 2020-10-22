Release\Stage2.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage2.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage2_tests1_test1.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 1024 1024 -samples 4
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x4_Stage2.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x4_RayTracerAss1.exe.bmp Outputs\stage2_tests1_test2.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 500 300 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_500x300x1_Stage2.exe.bmp Outputs_REFERENCE\cornell.txt_500x300x1_RayTracerAss1.exe.bmp Outputs\stage2_tests2_test3.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/allmaterials.txt -size 1000 1000 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\allmaterials.txt_1000x1000x1_Stage2.exe.bmp Outputs_REFERENCE\allmaterials.txt_1000x1000x1_RayTracerAss1.exe.bmp Outputs\stage2_tests1_test4.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/cornell-199lights.txt -size 256 256 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell-199lights.txt_256x256x1_Stage2.exe.bmp Outputs_REFERENCE\cornell-199lights.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage2_tests1_test5.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/5000spheres.txt -size 480 270 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\5000spheres.txt_480x270x1_Stage2.exe.bmp Outputs_REFERENCE\5000spheres.txt_480x270x1_RayTracerAss1.exe.bmp Outputs\stage2_tests2_test6.bmp
Release\Stage2.exe -runs 1 -threads 8 -input Scenes/dudes.txt -size 256 256 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\dudes.txt_256x256x1_Stage2.exe.bmp Outputs_REFERENCE\dudes.txt_256x256x1_RayTracerAss1.exe.bmp Outputs\stage2_tests1_test7.bmp
cmd \k