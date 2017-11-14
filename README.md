# Team frAIburg - Audi Autonomous Driving Cup 2017
This repository contains team frAIburg's Code for the AudiCup 2017.

**If you participate in AADC2018 we highly recommend to read the
[short bullet list with notes on ADTF](docs/adtf_notes.md) and to checkout
our wrapper library [`adtf_slim`](aadcUser/lib_adtf_slim),
which simplifies pin creation and sending of ADTF native types
and also OpenCV types with no overhead.**



### Installation

Besides adtf, boost, and OpenCV, this code needs tensorflow (we tested with version 1.4),
qpoaisis and eigen. We expect them to live in the folder ADTF/Lib, if you place
them somewhere else make sure to adapt the CMakeLists.txt.

##### Tensorflow
To optimize tensorflow for the car, we recommend building it directly on the car.
In principal you can follow the official [guide](https://www.tensorflow.org/install/install_sources),
but instead of only building the pip package you also need to build the libtensorflow.so library
for the c api. The installation of the pip package is only necessary if you plan to execute the demo task
via the thrift server. The main autonomous mode uses the TF C-API to directly run the forward
pass from an existing buffer.
At the time of writing this, this could be achieved by
` bazel build --config=opt --config=cuda //tensorflow/c:c_api //tensorflow/tensorflow/ //tensorflow/tools/pip_package:build_pip_package`
.


### Documentation:
Most documentation and additional explanations are contained in `README.md` files
in the respective folders.


[Felix Plum](https://github.com/felixplum),
[Philipp Jund](https://github.com/philjd),
[Markus Merlinger](http://github.com/RedHeadM),
[Jan Bechtold](http://github.com/janbechtold),
[Lior Fuks](http://github.com/liorfuks).
