# Tensorflow Object detection installation

It works like explained on the internet, this is a summary of the important steps. 
Some online tutorials suggest to install tensorflow in an anaconda environment, which I did. 
It is probably not necessary.

### Installing Anaconda

Anaconda can be downloaded from https://www.anaconda.com/download/
We should use the python 3 version and then run its bash script to install.

### Create conda virtual environment

'conda create -n tensorflow_cpu pip python=3.6'

'conda activate tensorflow_cpu'

Python 3.8 is the latest version, so maybe it is smarter, to switch to 3.8.
But it works with 3.6.

### Install Tensorflow

Install Tensorflow version 2.0.0. A lot of compartability issues can occur if we don´t use exactly this version.
Also the compartability of Tensorflow and the Object detection Toolbox might get changed to a later Tensorflowversion later on.

'conda install tensorflow==2.0.0'

or if that doesn´t work use pip

'pip install tensorflow==2.0.0'

### Testing installation

In a new terminal we activate the environment

'conda activate tensorflow_cpu'

Test the installation by importing tensorflow in a python interpreter. (just type python in the terminal and then the code)

'python'

```python
import tensorflow as tf
hello = tf.constant('Hello, TensorFlow!')
sess = tf.Session()
print(sess.run(hello))
```

this is the official test message, they use to test the installation. 
If it works, it will print "Hello, Tensorflow" without any errors.
Exept maybe a warning, that we are using an old version.

### now we need to install a lot of little programs

'conda install pillow, lxml, jupyter, matplotlib, opencv, cython'

maybe also contextlib2, I have it installed, but don´t know if it is needed.

### clone the repository

Somwhere in the home folder create a folder called tensorflow.
If you want to use my absolute paths use '~/Documents/tensorflow'
Clone the Tensorflow Models repository into this folder.
https://github.com/tensorflow/models
There should be a folder called 'tensorflow/models/research' now

### protobuf compiler usage

```
# From tensorflow/models/research/
wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip
unzip protobuf.zip

# From tensorflow/models/research/
./bin/protoc object_detection/protos/*.proto --python_out=.
```

### Add Environment Variables

'export PYTHONPATH=$PYTHONPATH:<PATH_TO_TF>/TensorFlow/models/research/object_detection'

```
# From within tensorflow/models/research/
export PYTHONPATH=$PYTHONPATH:<PATH_TO_TF>/TensorFlow/models/research:<PATH_TO_TF>/TensorFlow/models/research/slim
```
remember to swap <PATH_TO_TF> with the absolute path.

### Run the program

Go to the folder object detection
Save the sofar-test.py file in this folder
Run it from this folder, after you changed the paths in the document to the correct ones.
It uses any picture in the test images folder. Since the save command always uses the same name to save only one picture at a time can be safed. 
















