# Supplementary Codes for JESTPE-2023-12-1537

## Dependency Installation

The program requires the following free softwares.

- gcc (comes with [Minimalist GNU for Windows](https://sourceforge.net/projects/mingw/)---See this [awesome page](https://www3.ntu.edu.sg/home/ehchua/programming/howto/Cygwin_HowTo.html) for info)

- Anaconda3 (python), use pip to install following packages:
    - numpy
    - matplotlib
    - pandas

- gmake.exe (I use the one from [TI's CCS](https://www.ti.com/tool/download/CCSTUDIO), it is located somewhere like: `D:\ti\ccs930\ccs\utils\bin\gmake.exe`. You don't need to get it yourself, because I have copied and pasted `gmake.exe` to this repository.

## Complie Method 

- git clone https://github.com/horychen/FluxEstimatorsCodes.git (**[FluxEstimatorsCodes](https://github.com/horychen/FluxEstimatorsCodes)**)
- conda activate your-env-name 
- cd D:\your-codefile\\...\FluxEstimatorsCodes
- **Run code: python Codes/guiv2/main.py**
