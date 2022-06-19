# Biped (Pybullet)

## Setup

### Conda Environment Setup

To install Anaconda follow the instructions in the following webpage:  
https://www.digitalocean.com/community/tutorials/how-to-install-the-anaconda-python-distribution-on-ubuntu-20-04-quickstart

Create a conda environment for the PyBullet tutorial:  
```
$ conda create --name biped-pyb  
```
Switch to the newly create environment (you will notice the name of the environment on the command line in the extreme left):  
```
$ conda activate biped-pyb  
```

Once in the desired environment install the following packages:  
```
$ conda install nb_conda_kernels  
```

Install PyBullet (while in the environment):  
```
$ pip install pybullet  
```

Install Matplotlib (while in the environment):
```
$ conda install matplotlib
```

### Checking Pybullet Installation

To check the installation launch:  
```
$ python  
```

Inside the python environment import the pybullet and matplotlib libraries:  
```
>> import pybullet
>> import matplotlib
```
If this command executes without any error then the installation is successful.  

### Running Biped Code

Then, clone the repository on your system:
```
git clone https://github.com/tayalmanan28/Biped-Pybullet.git
```
Install the following required packages:
```
pip install -r requirements.txt
```


## Things to be done:

- [ ] add arg parse to switch between 
- [ ] Change colour of Biped
