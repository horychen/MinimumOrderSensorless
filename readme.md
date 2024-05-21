# Supplementary Codes for JESTPE3404227

## Dependency Installation

The program requires the following free softwares (tested in Windows 10).

- `gcc.exe` (comes with [Minimalist GNU for Windows](https://sourceforge.net/projects/mingw/)---See this [awesome page](https://www3.ntu.edu.sg/home/ehchua/programming/howto/Cygwin_HowTo.html) for info)

- `gmake.exe` (I use the one from [TI's CCS](https://www.ti.com/tool/download/CCSTUDIO), it is located somewhere like: `D:\ti\ccs930\ccs\utils\bin\gmake.exe`. You don't need to get it yourself, because I have copied and pasted `gmake.exe` to this repository.

- Anaconda3 (python), use pip to install following packages:
    - numpy
    - matplotlib
    - pandas

## Complie Method

- git clone https://github.com/horychen/MinimumOrderSensorless.git (**[MinimumOrderSensorless](https://github.com/horychen/MinimumOrderSensorless)**)

- Open `cmd.exe` in the directory where the codes locate, and execute commands:
    - `cd acmsimcv5/c`
    - `gmake.exe`
    - `main.exe`
    - `cd ..`
    - `cd ..`
    - `conda activate your-vitual-env` (optinal)
    - `python ACMPlot2024.py`

![readmeVisualized](https://github.com/horychen/MinimumOrderSensorless/blob/main/readme.png?raw=true)

- If you have trouble compling the C codes, it is also okay to directly execute `acmsimcv5/c/main.exe` and make sure the `*.dat` files are generated in `acmsimcv5/dat`.
    - It is a bad practice to git track `*.exe` file but I did it anyway just in case.
