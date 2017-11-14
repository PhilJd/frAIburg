Follow the installation instructions for the object detection api under https://github.com/tensorflow/models/blob/master/object_detection/g3doc/installation.md.

Make sure to add slim to the python path:
```export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim```
Run the following command to train the model:
```
python ./object_detection/train.py --logtostderr --train_dir=/mnt/data/training_checkpoints/coco_dolls_rcnn101_frcnn50/ --pipeline_config_path=/home/jundp/github/frAIburg_audicup/models/faster_rcnn_resnet101_coco.config
```