# Neural Network based Object Classifier

### Install dependencies : 

Requires ROS for running

```bash
sudo apt-get install --no-install-recommends libboost-all-dev libgflags-dev libgoogle-glog-dev liblmdb-dev libprotobuf-dev libleveldb-dev libsnappy-dev libhdf5-serial-dev protobuf-compiler libatlas-base-dev
```
### Build Caffe : 

```bash
roscd aerial_detection
cd caffe
cp Makefile.config.example Makefile.config
```
For CPU-only Caffe, uncomment CPU_ONLY := 1 in Makefile.config. 
```bash
nano Makefile.config
```
```bash
make all
make distribute
# hack for build issues
protoc src/caffe/proto/caffe.proto --cpp_out=.
mkdir include/caffe/proto
mv src/caffe/proto/caffe.pb.h include/caffe/proto
```

### Build package
```bash
cd ~/catkin_ws/
catkin build aerial_detection # or catkin_make
```
