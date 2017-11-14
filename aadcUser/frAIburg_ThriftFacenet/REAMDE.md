## Thrift Client for Facenet (Demo Task)

This is the ADTF filter that connects to the facenet python server.
It's forked off from the adtf thrift demo named ExtPythonServer.
(Which actually is a client, despite its name.)

### Installation
To install facenet, clone my fork of facenet into
this folder:
```git clone https://github.com/PhilJd/facenet.git```
and download the
[pretrained model](https://drive.google.com/file/d/0B5MzpY9kBtDVOTVnU3NIaUdySFE)
and save it under `/home/aadc/ADTF/weights/facenet`.
If you save it in a different directory make sure you edit the
`MODEL` variable in `facenet_interface.py`.

Generate the thrift headers:
```
thrift -r -gen cpp ExtIf.thrift
thrift -r -out python/gen-py/ -gen py ExtIf.thrift
```

### Debug hints:
- `No handlers could be found for logger "thrift.server.TServer"`: Can occur
    when python-thrift and system thrift versions mismatch. Check system thrift version
    with `thrift --version`. The output might look like this: Thrift version 0.9.1)
    Install the corresponding python version `pip install thrift==0.9.1'.
- Address already in use `kill -9 $(lsof -ti tcp:1833)` #port 1833

### Side Note:
As the input and output is the same for facenet and object detection, i.e.
`input=image, output=[bounding_boxes, labels]`, we also use this
server to measure the speed difference compared to our
cpp-tensorflow controller.
