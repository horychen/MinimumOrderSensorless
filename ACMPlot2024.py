import os, re, pandas as pd
from pylab import np, plt, mpl

# 风格
mpl.style.use('ggplot') # further customize: https://stackoverflow.com/questions/35223312/matplotlib-overriding-ggplot-default-style-properties and https://matplotlib.org/2.0.2/users/customizing.html
# plt.style.use("dark_background")
# mpl.style.use('grayscale')
# mpl.style.use('classic')

# 字体
mpl.rcParams['mathtext.fontset'] = 'stix'
mpl.rcParams['font.family'] = 'STIXGeneral'
plt.rcParams['font.weight'] ='900'     # bold is 700, normal is 400, see https://matplotlib.org/2.0.2/users/customizing.html
plt.rcParams['font.size']   =12 # 13 is too big, commenting out is too small

# 颜色设置壹/叁
# https://matplotlib.org/2.0.2/users/customizing.html from https://stackoverflow.com/questions/35223312/matplotlib-overriding-ggplot-default-style-properties
# plt.rcParams['axes.labelsize']  =14   # 'medium' # 'large'是没用的！
hex_monokai   = '#272822'
hex_wanderson = '#3d444c'
plt.rcParams['axes.facecolor']  =hex_wanderson # also need to change facecolor in mplwidget.py to get a consistent look
plt.rcParams['axes.labelcolor'] ='#d5d1c7' # tint grey
plt.rcParams['axes.axisbelow']  ='False'   # 'line'
plt.rcParams['ytick.color']     ='#d5d1c7' #'white'
plt.rcParams['xtick.color']     ='#d5d1c7' #'white'
plt.rcParams['text.color']      ='#d5d1c7' #'white'
plt.rcParams['grid.color']      ='#d5d1c7' # grid color
plt.rcParams['grid.linestyle']  ='--'      # solid
plt.rcParams['grid.linewidth']  =0.3       # in points
plt.rcParams['grid.alpha']      =0.8       # transparency, between 0.0 and 1.0


# 实用函数
import subprocess
def subprocess_cmd(command_string, bool_run_parallel=False):
    process = subprocess.Popen(command_string, stdout=subprocess.PIPE, shell=True)
    if not bool_run_parallel:
        streamdata = proc_stdout = process.communicate()[0].strip()
        print(proc_stdout)
    return process
def cyclic_generator(list_of_things):
    N, i = len(list_of_things), 0
    while True:
        yield list_of_things[i]
        i +=1
        if i > (N-1):
            i = 0

# 画图风格
cjh_linestyles= [
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.', 
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.', 
        '-', '--', (0, (3, 1, 1, 1)), ':', '-.', 
    ]
cjh_linestyle = cyclic_generator(cjh_linestyles)
# 颜色设置叁/叁
cjh_colors = [
        '#ff6347', # tomato red
        'white',   # #d5d1c7',
        '#B3F5FF', # dark-theme-bright-blue
        '#FFF0B3', # dark-theme-bright-yellow
        '#ffabff', # pink
        '#E1C7E0', # dark-theme-bright-violet(purple)
        '#ABF5D1', # dark-theme-bright-green
        '#FFBDAD', # dark-theme-bright-red
        '#00dcff', # blue
        '#c593ff', # dark-theme-bright-color1
        '#02dac3', # dark-theme-bright-color2
        '#efc9a1', # dark-theme-bright-color3
        # 'black',
        'blue',
        '#857F72', # warmGrey
        '#616E7C', # Cool Grey
        '#B8B2A7', # warmGrey
        '#9AA5B1', # Cool Grey
        '#504A40', # warmGrey
        '#27241D', # warmGrey
    ] # https://uxdesign.cc/dark-mode-ui-design-the-definitive-guide-part-1-color-53dcfaea5129
cjh_color = cyclic_generator(cjh_colors)
# 注意，不可以做 list(self.cjh_color)，因为这是一个无止境循环的发生器，会卡住的。。。




# 读取数据
if not os.path.exists('./acmsimcv5/dat'):
    os.makedirs('./acmsimcv5/dat')
data_file_name = r"./acmsimcv5/dat/IM_Marino05_IFE_Comparison-225-1000-4-5019.dat"
df_profiles = pd.read_csv(data_file_name, na_values = ['1.#QNAN', '-1#INF00', '-1#IND00'])
no_samples = df_profiles.shape[0]
no_traces  = df_profiles.shape[1]


DOWN_SAMPLE = 1
CL_TS = 1e-4
time = np.arange(1, no_samples+1) * DOWN_SAMPLE * CL_TS


def decode_labelsAndSignals():
    # #define DATA_LABELS

    with open('labels_im_user11.txt', 'r', encoding='utf-8') as f:
        buf = f.read()
        labels = [el.strip() for el in buf.split('\n') if el.strip()!='']
        # avoid using ',' or ';' in label, because comma will be interpreted as new column by pandas
        # labels = [el.replace(',','|') for el in labels]
        # print(labels)
    with open('signals_im_user11.txt', 'r', encoding='utf-8') as f:
        buf = f.read()
        details_ori = buf
        details_per_subplot = [el.strip() for el in buf.split('\n') if el.strip()!='']
        # print(details)

    # 每个通道有几条信号？
    list__number_of_traces_per_subplot = []
    for detail in details_per_subplot:
        number_of_traces_per_subplot = len( [el.strip() for el in detail.split(',') if el.strip()!=''] )
        list__number_of_traces_per_subplot.append(number_of_traces_per_subplot)

    if len(list__number_of_traces_per_subplot) != len(labels):
        print('\t[Warning] missing label or plotting variable.')
        for i in range(len(list__number_of_traces_per_subplot) - len(labels)):
            labels.append(f'No title {i}')

    number_of_subplot = len(list__number_of_traces_per_subplot)

    # print(details)
    return details_ori, list__number_of_traces_per_subplot, labels

details_ori, list__number_of_traces_per_subplot, list__label = decode_labelsAndSignals()
details = [el.strip() for el in re.split('\n|,', details_ori) if el.strip()!='']
number_of_subplot = len(list__number_of_traces_per_subplot)
number_of_subplot = 5

# for i, key in enumerate(df_profiles.keys()):
# for i, key in enumerate(self.list__label):
# 遍历子图们
trace_counter = 0
first_ax = None
for index, number_of_traces_per_subplot in enumerate(list__number_of_traces_per_subplot):
    if index % number_of_subplot == 0:
        plt.figure(facecolor='#3d444c')
        first_ax = None
    ax = plt.gcf().add_subplot(number_of_subplot, 1, 1+index - number_of_subplot*(index//number_of_subplot), sharex=first_ax)
    if first_ax is None:
        first_ax = ax

    # 遍历某一张子图中的波形们
    for j in range(number_of_traces_per_subplot):
        key = details[trace_counter]
        # print('key:', key)

        try:
            signal = df_profiles[key]
        except Exception as e:
            print('debug:', df_profiles.keys())
            raise e
        trace_counter += 1
        ax.plot(time, signal, linestyle=cjh_linestyles[j], color=cjh_colors[j], lw=1.5, label=key, alpha=0.5, zorder=100+trace_counter) # 如果是减去trace_counter，那么就是后来的画在下面

    # print(index)
    # print('DEBUG:', list__label[index], index, list__label)
    ax.set_ylabel(list__label[index])
    ax.legend(loc='lower right').set_zorder(202)
    ax.grid(True)
ax.set_xlabel('Time [s]')

plt.show()
axes = plt.gcf().get_axes()

