# n_Rays = [50, 100, 200, 400, 800, 1600, 3200]
# n_Particles = [50, 100, 200, 400, 800, 1600, 3200]

#  /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py 50 50
#  /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py 50 100
# ...
#  /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py 100 50
#  /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py 100 100
# ...
# /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py 3200 3200
# Code here

# for n_Rays in 50 100 200 400 800 1600 3200
for n in 0 1 2 3 4 5 6 7 8 9
do
    # for n_Particles in 50 100 200 400 800 1600 3200
    for lambda in 0.0, 0.25, 0.5, 0.75, 1.0
    do
        /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py $lambda $n
    done
done

# Remember to chmod by 