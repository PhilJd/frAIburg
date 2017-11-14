 TF_TYPE="gpu" # Change to "gpu" for GPU support
 OS="linux" # Change to "darwin" for Mac OS
 TARGET_DIRECTORY="/home/aadc/ADTF/Libs/tensorflow-1.2.1/"
    curl -L "https://storage.googleapis.com/tensorflow/libtensorflow/libtensorflow-${TF_TYPE}-${OS}-x86_64-1.2.1.tar.gz" |sudo tar -C $TARGET_DIRECTORY -xz
