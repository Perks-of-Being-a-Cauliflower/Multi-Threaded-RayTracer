Release\Stage1.exe -runs 1 -threads 8 -input Scenes/cornell.txt -size 500 300 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_500x300x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_500x300x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test3.bmp
Release\Stage1.exe -runs 1 -threads 8 -input Scenes/5000spheres.txt -size 480 270 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\5000spheres.txt_480x270x1_Stage1.exe.bmp Outputs_REFERENCE\5000spheres.txt_480x270x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test6.bmp
Release\Stage1.exe -runs 1 -threads 3 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test1a.bmp
Release\Stage1.exe -runs 1 -threads 5 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test1b.bmp
Release\Stage1.exe -runs 1 -threads 6 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test1c.bmp
Release\Stage1.exe -runs 1 -threads 7 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test1d.bmp
Release\Stage1.exe -runs 1 -threads 513 -input Scenes/cornell.txt -size 1024 1024 -samples 1
"c:\Program Files\ImageMagick-7.0.10-Q16-HDRI\magick.exe" compare -metric mae Outputs\cornell.txt_1024x1024x1_Stage1.exe.bmp Outputs_REFERENCE\cornell.txt_1024x1024x1_RayTracerAss1.exe.bmp Outputs\stage1_tests2_test1e.bmp
cmd \k