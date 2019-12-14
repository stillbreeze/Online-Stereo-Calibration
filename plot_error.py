import os
import numpy as np
from decimal import Decimal
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

import ipdb
st = ipdb.set_trace

result_folder = 'results'
exp_root = 'experiment_runners/exp_files'
units = 'deg'
folders = os.listdir(exp_root)
if not os.path.exists(result_folder):
    os.makedirs(result_folder)

information_metric = {}
for folder_path in folders:
    exp_path = os.path.join(exp_root,folder_path)
    if not os.path.isdir(exp_path):
        continue
    print(folder_path)
    errors = []
    feature_count = []
    # Read data
    for file in os.listdir(exp_path):
        if file.startswith('error') and file.endswith('.txt'):
            with open(os.path.join(exp_root, folder_path, file)) as f:
                content = f.readlines()
            file_data = []
            feature_count.append(float(content[-1].strip()))
            for err in content[:6]:
                file_data.append(float(err.strip()))
            file_data[:3] = file_data[:3][::-1]
            file_data = np.asarray(file_data)
            errors.append(file_data)

    a_opt = []
    d_opt = []
    e_opt = []
    reprojection_err = []
    epipolar_err = []
    for file in os.listdir(exp_path):
        if file.startswith('information') and file.endswith('.txt'):
            with open(os.path.join(exp_root, folder_path, file)) as f:
                content = f.readlines()
            file_data = []
            for metric in content:
                file_data.append(float(metric.strip()))
            a_opt.append(file_data[0])
            d_opt.append(file_data[1])
            e_opt.append(file_data[2])
            reprojection_err.append(file_data[3])
            epipolar_err.append(file_data[4])

    errors = np.asarray(errors)
    if units == 'deg':
        errors[:, :3] = (errors[:, :3] * 180) / np.pi
    a_opt = np.asarray(a_opt)
    d_opt = np.asarray(d_opt)
    e_opt = np.asarray(e_opt)
    reprojection_err = np.asarray(reprojection_err)
    epipolar_err = np.asarray(epipolar_err)
    epsilon = 1e-8
    information_metrics = np.asarray([np.log(a_opt+epsilon), np.log(d_opt+epsilon), np.log(e_opt+epsilon), reprojection_err, epipolar_err]).T
    feature_count = np.asarray(feature_count)
    total_samples = errors.shape[0]
    total_samples_metric = a_opt.shape[0]
    std = np.nanstd(errors, 0)
    mean = np.nanmean(errors, 0)

    std_metric = np.nanstd(np.ma.masked_invalid(information_metrics), 0)
    mean_metric = np.nanmean(np.ma.masked_invalid(information_metrics), 0)
    # max_val = mean.max() + 3*std.max()
    # min_val = mean.min() - 3*std.max()
    # y_range = max(np.abs(max_val), np.abs(min_val))

    # Plot
    fig, ax = plt.subplots(figsize=(18,15))
    np.set_printoptions(precision=5)
    # plt.ylim((-y_range, y_range))
    plt.ylim((-1, 1))
    plt.xlim((0,7.2))
    x = np.arange(1,7)
    label = ['Rx','Ry','Rz','tx','ty','tz']
    ax.set_xticks(x)
    ax.set_xticklabels(label, minor=False)
    ax.tick_params(labelsize=20)
    if units == 'deg':
        label_text = '(deg and m)'
    else:
        label_text = '(rad and m)'
    plt.ylabel('Error Bar ' + label_text, fontsize=20)
    plt.title('Errors from {} samples'.format(total_samples) + '\n\n Mean feature count = ' + str(feature_count.mean()) + '\n', fontsize=20)
    plt.errorbar(x, mean, std, linestyle='None', color='green', marker='o', ecolor='blue')
    for i, xidx in enumerate(x):
        mean_i = '%.2E' % Decimal(mean[i])
        std_i = '%.2E' % Decimal(std[i])
        offset = 0.02
        if units == 'deg':
            offset = 0.06
        yidx_mean = max(mean[i] + 1.3*std[i], mean[i] + offset)
        yidx_std = min(mean[i] - 1.3*std[i], mean[i] - offset)
        ax.text(xidx, yidx_mean, str(mean_i), color='green', fontsize=17)
        ax.text(xidx, yidx_std, str(std_i), color='blue', fontsize=17)
    legend_elements = [Line2D([0], [0], color='green', lw=1, label='Mean'),
                       Line2D([0], [0], color='blue', lw=1, label='Std')]
    ax.legend(handles=legend_elements, loc='best', prop={'size': 23})
    plot_path = os.path.join(result_folder, 'error_plot_' + folder_path + '.png')
    plt.savefig(plot_path, dpi=300)
    plt.close()
    information_metric[folder_path] = [mean_metric, std_metric]



metrics_list = ['a-opt', 'd_opt', 'e_opt', 'reprojection_err', 'epipolar_err']
for optimality_metric, data_idx in zip(metrics_list, range(len(metrics_list))):
    row_format = '{:>35}' * (3)
    print(row_format.format(optimality_metric, 'Mean', 'Std'))
    for experiment_name, value in sorted(information_metric.items()):
        print(row_format.format(experiment_name, value[0][data_idx], value[1][data_idx]))
    print('\n')

with open(os.path.join(result_folder, 'information_content_results.txt'), 'w') as text_file:
    for optimality_metric, data_idx in zip(metrics_list, range(len(metrics_list))):
        row_format = '{:>35}' * (3)
        text_file.write('-'*35*3)
        text_file.write('\n')
        text_file.write(row_format.format(optimality_metric, 'Mean', 'Std'))
        text_file.write('\n')
        text_file.write('-'*35*3)
        text_file.write('\n')
        for experiment_name, value in sorted(information_metric.items()):
            text_file.write(row_format.format(experiment_name, value[0][data_idx], value[1][data_idx]))
            text_file.write('\n')
        text_file.write('\n')



mid_list = np.arange(len(metrics_list))
color_list = ['--ro', '--bo', '--go', '--co', '--mo', '--yo']
optim_name_list = ['A-optimality', 'D-optimality', 'E-optimality', 'Reprojection Err', 'Epipolar Err']
# -------------------------------------
en = 'measurement_noise'
plot_text = 'Measurement Noise (in pixels)'
# -------------------------------------

for mid, optim_name in zip(mid_list, optim_name_list):
    label = []
    val = []
    for exp_name, metric_data in information_metric.items():
        if en in exp_name:
            label.append(int(exp_name.split('_')[-1]))
            val.append(metric_data[0][mid])
    label, val = zip(*sorted(zip(label, val)))
    plt.plot(label, val, color_list[mid], label=optim_name)
    plt.xticks(label, label)
    plt.xlabel(plot_text, fontsize=10)
    plt.ylabel('optimality metric', fontsize=10)
    plot_path = os.path.join(result_folder, en + '_' + optim_name + '_.png')
    plt.legend()
    plt.savefig(plot_path, dpi=300)
    plt.close()


# -------------------------------------
en = 'extrinsic_deviation'
plot_text = 'Extrinsic Deviation (in radians)'
# -------------------------------------

for mid, optim_name in zip(mid_list, optim_name_list):
    label = []
    val = []
    for exp_name, metric_data in information_metric.items():
        if en in exp_name:
            label.append(int(exp_name.split('_')[-1])*0.01)
            val.append(metric_data[0][mid])
    label, val = zip(*sorted(zip(label, val)))
    plt.plot(label, val, color_list[mid], label=optim_name)
    plt.xticks(label, label)
    plt.xlabel(plot_text, fontsize=10)
    plt.ylabel('optimality metric', fontsize=10)
    plot_path = os.path.join(result_folder, en + '_' + optim_name + '_.png')
    plt.legend()
    plt.savefig(plot_path, dpi=300)
    plt.close()


# -------------------------------------
en = 'feature_count'
plot_text = 'Average Feature Count'
# -------------------------------------

for mid, optim_name in zip(mid_list, optim_name_list):
    label = []
    val = []
    for exp_name, metric_data in information_metric.items():
        if en in exp_name:
            label.append(int(exp_name.split('_')[-1])//17)
            val.append(metric_data[0][mid])
    label, val = zip(*sorted(zip(label, val)))
    plt.plot(label, val, color_list[mid], label=optim_name)
    plt.xticks(label, label)
    plt.xlabel(plot_text, fontsize=10)
    plt.ylabel('optimality metric', fontsize=10)
    plot_path = os.path.join(result_folder, en + '_' + optim_name + '_.png')
    plt.legend()
    plt.savefig(plot_path, dpi=300)
    plt.close()


st()


