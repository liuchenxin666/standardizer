# standardizer

a system for calibrating medical equipment

## Usage
If you want to run this system:

First, install the requirements
```sh
conda create -n myqt python=3.8
conda activate myqt
pip install -r requirements.txt
```
Then, run the `initial.py` in the Pycharm, or

```sh
python3 initial.py
```
## UI

You can install the [PyQt5](https://blog.csdn.net/baidu_35145586/article/details/108110236):

open the `./ui/*.ui` by the designer.exe

## Hardware

`ADS1256.py`, `DAC8532.py`, `BME285.py` and `config.py` are related to Raspberry Pi hardware
