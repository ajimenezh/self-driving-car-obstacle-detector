set -x #echo on

cython --embed ros_node.pyx

gcc -O3 -I/usr/include/python2.7 -o ros_node ros_node.c -lpython2.7 -lpthread -lm -lutil -ldl

set +x
