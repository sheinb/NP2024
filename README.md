# Signal processing and analysis with Arduino and Python

## Arduino code
In the Arduino folder there are sketches for the Peripheral and Central ESP32 Feathers used to acquire remote data and send to central system for processing.

## Python code

To visualize and store the serial data obtained from the Central processor, there is a python script called `emgGUI.py` that can be run from an Anaconda environment with Pyside6 and Pyqtgraph installed.  A suitable environment can be created by:

```
conda create --name NP2024 numpy pandas seaborn pyserial
conda activate NP2024
pip install pyside6 pyside6-essentials pyside6-addons pyqtgraph
```

