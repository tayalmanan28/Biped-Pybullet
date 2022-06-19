# Biped (Pybullet)

## Setup

### Conda Environment Setup

To install Anaconda follow the instructions in this [webpage](https://www.digitalocean.com/community/tutorials/how-to-install-the-anaconda-python-distribution-on-ubuntu-20-04-quickstart)

Create a conda environment for the PyBullet tutorial:  
```
$ conda create --name biped-pyb  
```
Switch to the newly create environment (you will notice the name of the environment on the command line in the extreme left):  
```
$ conda activate biped-pyb  
```

Then, clone the repository on your system:
```
git clone https://github.com/tayalmanan28/Biped-Pybullet.git
```
Install the following required packages:
```
pip install -r requirements.txt
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

## Running the code
The Biped can walk, stand, squat, jump with Twist, jump without Twist and Twist its torso

So, to get the bot to Walk give the following command:
```
python3 main.py --action='walk'
```

To get the bot to do Squats give the following command:
```
python3 main.py --action='squat'
```

To get the bot to Jump with Torso Twist give the following command:
```
python3 main.py --action='jump_w_Twist'
```

To get the bot to Jump without Torso Twist give the following command:
```
python3 main.py --action='jump_wo_Twist'
```

To get the bot to do Torso Twist give the following command:
```
python3 main.py --action='torsoTwist'
```
To get the bot to just Stand at one place give the following command:
```
python3 main.py --action='stand'
```


## Things to be done:

- [ ] Change colour of Biped
