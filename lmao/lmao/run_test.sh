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

for n_Rays in 50 100 200 400 800 1600 3200
do
    for n_Particles in 50 100 200 400 800 1600 3200
    do
        /bin/python3 /home/junge/master_ws/src/Danitech-master/lmao/lmao/LocalizerNoRos.py $n_Rays $n_Particles
    done
done

# Remember to chmod by 