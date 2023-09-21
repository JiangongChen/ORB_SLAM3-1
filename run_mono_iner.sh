#./Examples/Monocular-Inertial/mono_inertial_tum_vi ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/TUM-VI.yaml ~/Dataset/tumVI/dataset-room4_512_16/mav0/cam0/data ./Examples/Monocular-Inertial/TUM_TimeStamps/dataset-room4_512.txt ./Examples/Monocular-Inertial/TUM_IMU/dataset-room4_512.txt
#./Examples/Monocular-Inertial/mono_inertial_tum_vi ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/TUM-VI.yaml ~/Dataset/tumVI/dataset-room1_512_16/mav0/cam0/data ./Examples/Monocular-Inertial/TUM_TimeStamps/dataset-room1_512.txt ./Examples/Monocular-Inertial/TUM_IMU/dataset-room1_512.txt
#export TAR_PATH=dataset-room4_512
#./Examples/Monocular-Inertial/mono_inertial_tum_vi ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/TUM-VI.yaml ~/Dataset/tumVI/${TAR_PATH}_16/mav0/cam0/data ./Examples/Monocular-Inertial/TUM_TimeStamps/$TAR_PATH.txt ./Examples/Monocular-Inertial/TUM_IMU/$TAR_PATH.txt
#export TAR_PATH=~/kalibr_workspace/pixel4/test8
#./Examples/Monocular-Inertial/mono_inertial_pixel ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/PIXEL4_LANDSCAPE.yaml $TAR_PATH/cam0 $TAR_PATH/timestamp.txt $TAR_PATH/imu0.csv
#./Examples/Monocular-Inertial/mono_inertial_edge ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/PIXEL4_LANDSCAPE.yaml
./Examples/Monocular-Inertial/mono_inertial_edge ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/PIXEL6.yaml