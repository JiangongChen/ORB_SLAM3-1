# transition error (SLAM unit), scale, transition error (Ground truth unit)
export TAR_PATH=./evaluation/room4
#python ./evaluation/evaluate_ate_scale.py $TAR_PATH/sync_gt.txt $TAR_PATH/allTrajectory0-cut.txt --plot $TAR_PATH/traj.pdf --save_scale $TAR_PATH/alignedTraj.txt
python ./evaluation/evaluate_ate_scale.py $TAR_PATH/sync_gt.csv $TAR_PATH/IMUTrajectory-cut-good.txt --plot $TAR_PATH/traj.pdf --save_scale $TAR_PATH/alignedTraj.txt
##python ./evaluation/evaluate_ate_scale.py ~/Dataset/tumVI/dataset-room1_512_16/mav0/mocap0/data.csv ./CameraTrajectory.txt --plot ./evaluation/traj.png --save ./evaluation/alignedTraj.txt