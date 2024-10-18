#!/usr/bin/env python3

# /home/roselab/miniconda3/envs/ml_rover/bin/python3
# -*- coding: utf-8 -*- #
"""
ʕっ•ᴥ•ʔっ
@authors: jen & sapph & river

last updated: aug 19 2024 09:22
"""

# ROS imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cr_interfaces.msg import MoisturePercent

# Basics
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as ss

# Specialized
from numpy.linalg import inv
from osgeo import gdal
from PIL import Image
import threading
import torch
import gpytorch
import sklearn.metrics
import cv2

# Other
from serial import Serial, SerialException
from scipy.stats import ks_2samp
from sklearn.metrics import mean_squared_error
import serial
import shutil
import pathlib
import glob
import datetime
import random
import csv
import pickle
import re
import math
import time
import sys
import argparse

parser = argparse.ArgumentParser(
    prog='FieldTestScript',
    description='Records data collected in a field test and performs active learning for a rover in a grid.',
    epilog='Text at the bottom of help')
parser.add_argument('TrialNumber', type=int, help='Trial Number')
parser.add_argument('KernelType', type=str,
                    help='Kernel Type: RBF, Matern')
args = parser.parse_args()

# Moisture sensor usage
MOISTURESENSOR = True
# Science blind vs. active learning
SNAKEPATTERN = False
SPIRALPATTERN = False
# Other
MOVEFILES = False

# Naming convention
TrialNumber = args.TrialNumber
TrialName = "AL_Trial"

# Field test grid sizing [100x100 square]
grid_length = 10
grid_width = 10

# ML parameters
kernel_type = args.KernelType
# kernel_type = 'RBF', 'Matern'
poly_rank = 4
r_disp = 6.0
constraint_flag = 1
local_flag = 1

if local_flag == 1:
    explore_name = 'local'
elif local_flag == 2:
    explore_name = 'NN'
else:
    explore_name = 'global'

if constraint_flag == 0:
    con_name = 'unconstrained'
else:
    con_name = 'constrained'

###############################################################################
############################# ROS Implementation ##############################
###############################################################################
class FieldTestNode(Node):
    def __init__(self):
        super().__init__('field_test_node')
        self.moisture_percent_sub = self.create_subscription(MoisturePercent, 'moisture_percent', self.moisture_percent_callback, 10)
        self.moisture_data = []
        self.data_lock = threading.Lock()
        self.TrialNumber = TrialNumber
        self.TrialName = TrialName
        self.grid_length = grid_length
        self.grid_width = grid_width
        self.kernel_type = kernel_type
        self.poly_rank = poly_rank
        self.r_disp = r_disp
        self.constraint_flag = constraint_flag
        self.local_flag = local_flag
        self.explore_name = explore_name
        self.con_name = con_name
        self.sigma = 0.000289305
        self.MOISTURESENSOR = MOISTURESENSOR
        self.SNAKEPATTERN = SNAKEPATTERN
        self.SPIRALPATTERN = SPIRALPATTERN
        self.MOVEFILES = MOVEFILES
        self.testedWaypoints = []
        self.count = 0
        self.iteration = 0
        self.sample_iter = None
        self.training_iter = 100
        self.i_train = []
        self.var_iter = []
        self.var_iter_local = []
        self.var_iter_global = []
        self.lengthscale = []
        self.covar_global = []
        self.covar_trace = []
        self.covar_totelements = []
        self.covar_nonzeroelements = []
        self.f2_H_global = []
        self.f2_H_local = []
        self.flags = []
        self.noise = []
        self.drift_values = []
        self.retroactive_error = []
        self.min_drift = float('inf')
        self.max_drift = float('-inf')
        self.samp = 1

    class ExactGPModel(gpytorch.models.ExactGP):
        # Define the simplest form of GP model, exact inference
        def __init__(self, train_x, train_y, likelihood, kernel_type, poly_rank):
            super().__init__(train_x, train_y, likelihood)
            self.mean_module = gpytorch.means.ConstantMean()

            if kernel_type == 'RBF':
                # RBF Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())
            elif kernel_type == 'Matern':
                # Matern Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.MaternKernel())
            elif kernel_type == 'Periodic':
                # Periodic Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.PeriodicKernel())
            elif kernel_type == 'Piece_Polynomial':
                # Piecewise Polynomial Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.PiecewisePolynomialKernel())
            elif kernel_type == 'RQ':
                # RQ Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RQKernel())
            elif kernel_type == 'Cosine':
                # Cosine Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.CosineKernel())
            elif kernel_type == 'Linear':
                # Linear Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.LinearKernel())
            elif kernel_type == 'Polynomial':
                # Polynomial Kernel
                self.covar_module = gpytorch.kernels.ScaleKernel(
                    gpytorch.kernels.PolynomialKernel(ard_num_dims=3, power=poly_rank))

        def forward(self, x):
            mean_x = self.mean_module(x)
            covar_x = self.covar_module(x)
            return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

    def moisture_percent_callback(self, msg: MoisturePercent):
        with self.data_lock:
            self.moisture_data.append(msg.moisture_percent)

    def RKHS_norm(self, K, y):
        n_row, n_col = K.shape
        alpha = inv(K + self.sigma ** 2 * np.eye(n_row))
        return alpha.transpose()

    def unique_sample(self, i_sample, i_set, i_max):
        i_train = self.i_train
        x = self.x_true
        if i_sample <= i_max and i_sample >= 0:
            if i_sample not in i_train:
                i_new = i_sample
            else:
                i_set_unique = set(i_set) - set(i_train)
                if not i_set_unique:
                    return None
                i_set_unique = list(i_set_unique)
                x_start = x[i_sample, :]
                x_disp = np.sqrt(
                    (x[i_set_unique, 0] - x_start[0]) ** 2 + (x[i_set_unique, 1] - x_start[1]) ** 2)
                i_new = i_set_unique[np.argmin(x_disp)]
        elif i_sample > i_max:
            i_new = self.unique_sample(i_sample - 1, i_set, i_max)
        else:
            i_new = self.unique_sample(i_sample + 1, i_set, i_max)
        return i_new

    def sample_disp_con(self, x_start, r_disp):
        x = self.x_true
        x_disp = np.sqrt((x[:, 0] - x_start[0]) ** 2 + (x[:, 1] - x_start[1]) ** 2)
        i_con = np.argwhere(x_disp <= r_disp)
        i_con = np.sort(i_con)
        return list(i_con[:, 0])

    def GPtrain(self):
        x_train = self.x_train
        y_train = self.y_train
        training_iter = self.training_iter
        # Initialize likelihood and model
        self.likelihood = gpytorch.likelihoods.GaussianLikelihood()
        self.model = self.ExactGPModel(x_train, y_train, self.likelihood, self.kernel_type, self.poly_rank)
        # Find optimal model hyperparameters
        self.model.train()
        self.likelihood.train()

        # Use the Adam optimizer
        self.optimizer = torch.optim.Adam([
            {'params': self.model.parameters()},  # Includes GaussianLikelihood parameters
        ], lr=0.1)

        # "Loss" for GPs - the marginal log likelihood
        mll = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.model)

        # Train GP model
        for i in range(training_iter):
            # Zero gradients from previous iteration
            self.optimizer.zero_grad()
            # Output from model
            self.output = self.model(x_train)
            # Calc loss and backprop gradients
            loss = -mll(self.output, y_train)
            loss.backward()
            self.optimizer.step()
        self.loss = loss

    def GPeval(self):
        x_test = self.x_test
        # Get into evaluation (predictive posterior) mode
        self.model.eval()
        self.likelihood.eval()

        # Make predictions by feeding model through likelihood
        with torch.no_grad(), gpytorch.settings.fast_pred_var():
            self.observed_pred = self.likelihood(self.model(x_test))

        f_preds = self.model(x_test)
        y_preds = self.likelihood(self.model(x_test))
        self.f_mean = f_preds.mean
        self.f_var = f_preds.variance
        self.f_covar = f_preds.covariance_matrix

        with torch.no_grad():
            # Get upper and lower confidence bounds
            self.lower, self.upper = self.observed_pred.confidence_region()

    def create_folder_with_suffix(self, folder_path):
        i = 1
        while True:
            new_folder_path = folder_path + f"_{i}" if i > 1 else folder_path
            if not os.path.exists(new_folder_path):
                os.makedirs(new_folder_path)
                return new_folder_path
            i += 1

    def snake_pattern(self):
        grid = self.grid
        result = []
        for i, row in enumerate(grid):
            if i % 2 == 0:
                result.extend(row)
            else:
                result.extend(reversed(row))
        return result

    def spiral_traversal(self):
        grid = [row[:] for row in self.grid]  # Make a deep copy
        result = []
        while grid:
            result.extend(grid.pop(0))  # Traverse top row
            if grid and grid[0]:
                for row in grid:
                    result.append(row.pop())  # Traverse right column
            if grid:
                result.extend(grid.pop()[::-1])  # Traverse bottom row in reverse
            if grid and grid[0]:
                for row in reversed(grid):
                    result.append(row.pop(0))  # Traverse left column
        return result

    def update_visualization(self, next_point):
        sampled_points = self.testedWaypoints[:-1]
        grid = np.zeros((self.grid_length, self.grid_width))
        ax5 = self.fig1.add_subplot(1, 1, 1)
        ax5.imshow(grid, cmap='gray', extent=[-0.5, self.grid_width - 0.5, -0.5, self.grid_length - 0.5])
        ax5.plot(np.array(next_point)[0], np.array(next_point)[1], marker='o', color='green')
        if sampled_points:
            ax5.scatter(np.array(sampled_points)[:, 0], np.array(sampled_points)[:, 1], marker='x', color='red')
        ax5.set_title('Rover Exploration')
        ax5.set_xlabel('X')
        ax5.set_ylabel('Y')
        ax5.set_xticks(np.arange(0, self.grid_width, 1))
        ax5.set_yticks(np.arange(0, self.grid_length, 1))
        ax5.grid(True)

        plt.pause(1)  # Pause to allow the plot to be displayed
        image_file_path = os.path.join(self.new_folder_path, "GPAL_Path.png")
        self.fig1.savefig(image_file_path)

    def plotGP(self):
        fig2 = self.fig2
        x_true = self.x_true
        i_train = self.i_train
        y_train = self.y_train
        x_test_global = self.x_test_global
        observed_pred_global = self.observed_pred_global
        x_test_local = self.x_test_local
        lower_local = self.lower_local
        upper_local = self.upper_local
        var_iter_local = self.var_iter_local
        var_iter_global = self.var_iter_global
        drift_values = self.drift_values
        retroactive_error = self.retroactive_error
        i_sample = self.i_sample
        samp = self.samp

        ax1 = fig2.add_subplot(2, 3, 1)
        fig2.figsize = (14, 10)

        ax1.plot(x_true[i_train, 0], x_true[i_train, 1], color='black')
        ax1.scatter(x_true[:, 0], x_true[:, 1], s=1, alpha=0.25)
        ax1.set_title('Rover Path [Exploration]', fontsize=10)
        ax1.set_xlabel('X [km]', fontsize=8)
        ax1.set_ylabel('Y [km]', fontsize=8)
        ax1.tick_params(axis='both', which='major', labelsize=8)
        ax1.tick_params(axis='both', which='minor', labelsize=8)

        ax2 = fig2.add_subplot(2, 3, 2, projection='3d')
        ax2.scatter3D(x_true[i_sample, 0], x_true[i_sample, 1],
                      y_train[samp - 1], s=100, color='black', marker='*', zorder=1)
        ax2.plot_trisurf(
            x_test_global[:, 0], x_test_global[:, 1], observed_pred_global.mean.detach().numpy(), cmap='inferno', linewidth=0,
            alpha=0.25, vmax=max(y_train), vmin=min(y_train))
        # Shade between the lower and upper confidence bounds
        for i_test in range(len(x_test_local)):
            ax2.plot(x_test_local[i_test, 0].detach().numpy() * np.array([1, 1]),
                     x_test_local[i_test, 1].detach().numpy() * np.array([1, 1]),
                     np.array([lower_local[i_test].detach().numpy(), upper_local[i_test].detach().numpy()]), 'gray')
        ax2.view_init(20, 20)
        ax2.set_xlabel('X', fontsize=8)
        ax2.set_ylabel('Y', fontsize=8)
        ax2.set_zlabel('Percentage Moisture [%]', fontsize=8)
        ax2.tick_params(axis='z', labelsize=8)
        ax2.set_title('Moisture Distribution \n[Model Prediction]', fontsize=10)
        ax2.tick_params(axis='both', which='major', labelsize=8)
        ax2.tick_params(axis='both', which='minor', labelsize=8)

        ax3 = fig2.add_subplot(2, 3, 4)
        ax3.plot(range(0, len(var_iter_local)), var_iter_local, color='blue', marker='.')
        ax3.plot(range(0, len(var_iter_global)), var_iter_global, color='black', marker='*')
        ax3.set_xlabel('Number of Samples', fontsize=8)
        ax3.set_ylabel('Variance', fontsize=8)
        ax3.set_title('Variance vs. Samples', fontsize=10)
        ax3.legend(['local', 'global'], loc='upper right')
        ax3.tick_params(axis='both', which='major', labelsize=8)
        ax3.tick_params(axis='both', which='minor', labelsize=8)

        ax4 = fig2.add_subplot(2, 3, 5)
        ax4.plot(range(0, len(drift_values)), drift_values, label='Local', color='black')
        ax4.set_xlabel('Number of Samples', fontsize=8)
        ax4.set_ylabel('Drift', fontsize=8)
        ax4.set_title('Model Drift vs. Samples', fontsize=10)

        ax5 = fig2.add_subplot(2, 3, 6)
        ax5.plot(range(0, len(retroactive_error)), retroactive_error, label='Global', color='black')
        ax5.set_xlabel('Number of Samples', fontsize=8)
        ax5.set_ylabel('Error in Moisture Estimate', fontsize=8)
        ax5.set_title('Retroactive Prediction \nError', fontsize=10)

    def move_txt_files(self, source_folder, destination_folder):
        # Ensure destination folder exists, create if not
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)

        # Iterate through files in source folder
        for filename in os.listdir(source_folder):
            if filename.endswith(".txt") or filename.endswith(".asd"):
                # Build paths for source and destination files
                source_file = os.path.join(source_folder, filename)
                destination_file = os.path.join(destination_folder, filename)

                # Move the file
                shutil.move(source_file, destination_file)
                print(f"Moved {filename} to {destination_folder}")

    def process_data(self):
        cont = False
        data_average = None

        ##################################################
        ########## Moisture Sensor Utilization ###########
        ##################################################
        if self.MOISTURESENSOR:
            try:
                # Reset the moisture data list
                with self.data_lock:
                    self.moisture_data = []

                # Record the start time
                start_time = time.time()

                # Wait for 100 seconds while collecting data
                while time.time() - start_time <= 5:
                    rclpy.spin_once(self, timeout_sec=0.1)

                # After 100 seconds, process the collected data
                with self.data_lock:
                    moisture_data_copy = self.moisture_data.copy()
                    self.get_logger().info(f"Data received from moisture sensor: {self.moisture_data}")
                    if not self.moisture_data:
                        raise ValueError("No data received from moisture sensor.")
                    data_average = sum(self.moisture_data) / len(self.moisture_data)

                csv_output_directory = self.new_folder_path
                with open(csv_output_directory + '/' + 'moisture_data_trial' + str(self.TrialNumber) + '_flag' + str(self.flag) + '.csv', 'a') as file:
                    file.write("Time,MoistureLevel\n")  # header of csv file
                    for data in moisture_data_copy:
                        file.write(f"0,{data}\n")
                print("Data collection complete. Processing data...")

                # Load data from reading
                data = pd.read_csv(
                    csv_output_directory + '/' + 'moisture_data_trial' + str(self.TrialNumber) + '_flag' + str(
                        self.flag) + '.csv')

                # Convert milliseconds to seconds
                moisture_data = data['MoistureLevel']
                data_average_sum = sum(moisture_data)
                data_average_len = len(moisture_data)
                data_average = data_average_sum / data_average_len

            except serial.SerialException as e:
                print(f"Error with serial communication: {e}")
                cont = True
                data_average = None
            except FileNotFoundError as e:
                print(f"Error with file operation: {e}")
                cont = True
                data_average = None
            except pd.errors.EmptyDataError as e:
                print(f"Error reading CSV data: {e}")
                cont = True
                data_average = None
            except Exception as e:
                print(f"An unexpected error occurred: {e}")
                cont = True
                data_average = None

        return cont, data_average

    def run(self):
        base_path = os.path.expanduser('~/field_testing_al')
        date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.new_folder_path = self.create_folder_with_suffix(
            base_path + "/" + date + "/" + self.TrialName + str(self.TrialNumber))
        self.parent_dir = self.new_folder_path
        self.image_path = self.parent_dir

        self.fig1 = plt.figure()

        ###############################################################################
        ######################### Defining space to be tested #########################
        ###############################################################################
        x_coordinates = np.arange(self.grid_length)
        y_coordinates = np.arange(self.grid_width)
        x_grid, y_grid = np.meshgrid(x_coordinates, y_coordinates)
        self.x_true = np.stack((x_grid, y_grid), axis=-1)
        x_true = self.x_true.tolist()
        x_true = [item for sublist in x_true for item in sublist]
        self.x_true = np.array(x_true)

        n = int(np.sqrt(len(self.x_true)))
        m = n
        self.grid = self.x_true.reshape((n, m, 2))

        num = 0
        coordinate_to_number = {}
        for i, row in enumerate(self.grid):
            for j, coord in enumerate(row):
                coordinate_to_number[tuple(coord)] = num
                num += 1

        # Data normalization/standardization
        x_center_all = np.mean(self.x_true, 0)
        x_disp = np.sqrt((self.x_true[:, 0] - x_center_all[0]) ** 2 + (self.x_true[:, 1] - x_center_all[1]) ** 2)
        i_min = np.argmin(x_disp)
        x_center = self.x_true[i_min, :]
        self.x_true = self.x_true - x_center
        ###############################################################################
        ########################### Constants for Training ############################
        ###############################################################################
        n_samples = len(self.x_true)
        self.r_NN = np.sqrt(1)
        self.r_con = self.r_NN
        i_seq = list(range(0, n_samples))

        # Hyperparameters for exploration training
        if self.SNAKEPATTERN or self.SPIRALPATTERN:
            self.sample_iter = self.grid_length * self.grid_width
        else:
            self.sample_iter = int(n_samples / 2)
        self.i_train = []
        self.var_iter = []
        self.var_iter_local = []
        self.var_iter_global = []
        self.lengthscale = []
        self.covar_global = []
        self.covar_trace = []
        self.covar_totelements = []
        self.covar_nonzeroelements = []
        self.f2_H_global = []
        self.f2_H_local = []
        self.flags = []

        ###############################################################################
        # Randomly sample next 10 data points with a displacement constraint of 10 int #
        ###############################################################################
        if self.SNAKEPATTERN:
            gridIndex = [[i + j * self.grid_length for i in range(self.grid_width)] for j in range(self.grid_length)]
            i_train_known = self.snake_pattern()
            self.i_train.append(i_train_known[0])
            i_sample = i_train_known[0]
        elif self.SPIRALPATTERN:
            gridIndex = [[i + j * self.grid_length for i in range(self.grid_width)] for j in range(self.grid_length)]
            i_train_known = self.spiral_traversal()
            self.i_train.append(i_train_known[0])
            i_sample = i_train_known[0]
        else:
            random.seed(None)  # 42
            i_0 = random.randrange(n_samples)  # Randomly initialize location
            self.i_train.append(i_0)
            for i in range(5):  # 5 total points
                i_sample_set = self.sample_disp_con(self.x_true[self.i_train[-1]], self.r_NN)
                i_sample = i_sample_set[random.randrange(len(i_sample_set))]
                self.i_train.append(int(i_sample))
            self.i_train = list(set(self.i_train))
            print("Number of initial points:", len(self.i_train))

        # Creating first 10 payload input values corresponding to above
        y_obs = np.array([])

        for coordinate in self.x_true[self.i_train, :]:
            coordinatePoint = coordinate + x_center
            self.testedWaypoints.append(coordinatePoint)

            coord = tuple(coordinatePoint.flatten())
            if coord in coordinate_to_number:
                self.flag = coordinate_to_number[coord]

            print("Rover Location --> Flag: " + str(self.flag) + " & Coordinate: " + str(coord))
            if self.count != 0:
                rover_path = [item.tolist() if isinstance(item, np.ndarray) else item for item in self.testedWaypoints]
                self.update_visualization(list(coordinatePoint))

            cont = True
            while cont:
                print("[T-Minus 20 sec until data is collected]")
                # time.sleep(10)  # Wait 20 seconds for rover to move to position & insert probe
                print("[T-Minus 10 sec until data is collected]")
                # time.sleep(5)
                print("[T-Minus 5 sec until data is collected]")
                # time.sleep(5)
                print("Collecting Data... [T-Minus 100 sec until completion]")
                cont, data_average = self.process_data()

            print("Average Moisture Percentage: ", data_average)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            y_obs = np.append(y_obs, float(data_average))
            file_path = os.path.join(self.new_folder_path, "GPAL_tested_coordinates.txt")
            testedCoord = open(file_path, "a")
            testedCoord.write(str(coordinatePoint) + '\n')
            testedCoord.close()
            file_path = os.path.join(self.new_folder_path, "GPAL_correlation_values.txt")
            moistVals = open(file_path, "a")
            moistVals.write(str(data_average) + '\n')
            moistVals.close()
            self.count += 1
            self.iteration += 1

        # Baseline model for prediction & drift calculations
        self.x_train = torch.from_numpy(self.x_true[self.i_train, :]).float()
        self.y_train = torch.from_numpy(y_obs).float()

        self.GPtrain()

        # Exploration Phase: loop to append sample and retrain GP model
        for j in range(self.sample_iter):
            # Define training data
            self.x_train = torch.from_numpy(self.x_true[self.i_train, :]).float()
            self.y_train = torch.from_numpy(y_obs).float()

            self.GPtrain()

            # Store optimal hyperparameters
            if self.kernel_type in ['RBF', 'Matern', 'Piece_Polynomial']:
                self.noise.append(self.model.likelihood.noise.detach().detach().numpy())
                self.lengthscale.append(self.model.covar_module.base_kernel.lengthscale.detach().detach().numpy()[0])

            # Validation set from observed data
            validation_samples = min(4, len(self.i_train))
            i_val = self.i_train[-validation_samples:]
            x_val = torch.from_numpy(self.x_true[i_val, :]).float()  # Validation set

            self.x_test = x_val
            self.GPeval()
            y_new = self.observed_pred.mean.detach().detach().numpy()
            y_baseline = y_obs[-validation_samples:]
            drift = np.mean(np.abs(y_baseline - y_new))  # Calculate model drift

            self.min_drift = min(self.min_drift, drift)
            self.max_drift = max(self.max_drift, drift)
            normalized_drift = (drift - self.min_drift) / (self.max_drift - self.min_drift) if self.max_drift > self.min_drift else 0
            self.drift_values.append(normalized_drift)

            # Test points are regularly spaced centered along the last index bounded by index displacement
            i_con = self.sample_disp_con(self.x_true[self.i_train[-1]], self.r_con)
            x_test_local = torch.from_numpy(self.x_true[i_con, :]).float()
            x_test_global = torch.from_numpy(self.x_true[i_seq, :]).float()

            self.x_test = x_test_local
            self.GPeval()
            self.observed_pred_local = self.observed_pred
            self.lower_local = self.lower
            self.upper_local = self.upper
            self.f_var_local = self.f_var
            self.x_test_local = x_test_local
            self.var_iter_local.append(max(self.f_var_local.detach().numpy()))

            self.x_test = x_test_global
            self.GPeval()
            self.observed_pred_global = self.observed_pred
            self.lower_global = self.lower
            self.upper_global = self.upper
            self.f_var_global = self.f_var
            self.x_test_global = x_test_global
            self.var_iter_global.append(max(self.f_var_global.detach().numpy()))

            # Evaluate covariance properties
            self.covar_global.append(self.f_covar)
            self.covar_trace.append(np.trace(self.f_covar.detach().detach().numpy()))
            self.covar_totelements.append(np.size(self.f_covar.detach().detach().numpy()))
            self.covar_nonzeroelements.append(np.count_nonzero(self.f_covar.detach().detach().numpy()))
            mll = gpytorch.mlls.ExactMarginalLogLikelihood(self.likelihood, self.model)

            # Evaluate RKHS norm
            K_global = self.output._covar.detach().detach().numpy()
            y_global = self.y_train.detach().numpy().reshape(len(self.y_train), 1)
            f2_H_sample = self.RKHS_norm(K_global, y_global)
            self.f2_H_global.append(f2_H_sample[0, 0])

            n_set = len(self.i_train)
            n_sub = math.floor(n_set / 2)
            i_sub = random.sample(range(1, n_set), n_sub)
            i_sub.sort()
            K_local = K_global[np.ix_(i_sub, i_sub)]
            y_local = y_global[i_sub]
            f2_H_sample = self.RKHS_norm(K_local, y_local)
            if j == 1 and self.SNAKEPATTERN and self.SPIRALPATTERN:
                self.f2_H_local.append(f2_H_sample[0, 0])

            # Pick the next point to sample by maximizing local variance and minimizing distance
            try:
                if self.SNAKEPATTERN or self.SPIRALPATTERN:
                    # Waypoint within r_con with maximum variance, nearest neighbor along the way
                    uncertainty = self.upper_local - self.lower_local
                    i_max = np.argmax(uncertainty)
                    x_max = self.x_test_local[i_max, :].detach().numpy()
                    i_NN = self.sample_disp_con(self.x_true[self.i_train[-1]], self.r_NN)
                    dx_NN = np.sqrt((self.x_true[i_NN, 0] - x_max[0]) ** 2 + (self.x_true[i_NN, 1] - x_max[1]) ** 2)
                    i_dx = np.argsort(dx_NN)
                    self.i_train.append(i_train_known[j + 1])
                    self.i_sample = i_train_known[j + 1]
                elif self.local_flag == 1 or self.local_flag == 2:
                    # Waypoint within r_con with maximum variance, nearest neighbor along the way
                    uncertainty = self.upper_local - self.lower_local
                    i_max = np.argmax(uncertainty)
                    x_max = self.x_test_local[i_max, :].detach().numpy()
                    next_point = self.x_true[i_max]
                    self.x_test = torch.tensor([next_point]).float()
                    self.GPeval()
                    y_pred_next = self.observed_pred.mean.detach().detach().numpy()[0]
                    i_NN = self.sample_disp_con(self.x_true[self.i_train[-1]], self.r_NN)
                    dx_NN = np.sqrt((self.x_true[i_NN, 0] - x_max[0]) ** 2 + (self.x_true[i_NN, 1] - x_max[1]) ** 2)
                    i_dx = np.argsort(dx_NN)
                    self.i_sample = []
                    jdx = 0
                    while not np.array(self.i_sample).size:
                        self.i_sample = self.unique_sample(i_NN[i_dx[jdx]], i_con, n_samples - 1)
                        count1 = 0
                        while self.i_sample is None:
                            count1 += 1
                            i_sample_set = self.sample_disp_con(self.x_true[self.i_train[-1]], np.sqrt(count1) - 0.29)
                            self.i_sample = i_sample_set[random.randrange(len(i_sample_set))]
                            if self.i_sample in self.i_train:
                                self.i_sample = None
                            print("**OH NO: NON-UNIQUE POINT**")
                        jdx += 1
                    self.i_train.append(int(self.i_sample))
                else:
                    # Waypoint within global space with maximum variance, directly go
                    uncertainty = self.upper_global - self.lower_global
                    i_max = np.argmax(uncertainty)
                    # Waypoint within entire space with max variance, nearest neighbor
                    if self.constraint_flag == 1:
                        a = self.x_test_global[i_max, :].detach().numpy()
                        i_NN = self.sample_disp_con(self.x_true[self.i_train[-1]], self.r_NN)
                        dx_NN = np.sqrt((self.x_true[i_NN, 0] - x_max[0]) ** 2 + (self.x_true[i_NN, 1] - x_max[1]) ** 2)
                        i_dx = np.argsort(dx_NN)
                        # Finds the nearest neighbor along the way to the point of highest variance
                        self.i_sample = []
                        jdx = 0
                        while not np.array(self.i_sample).size:
                            self.i_sample = self.unique_sample(i_NN[i_dx[jdx]], i_con, n_samples - 1)
                            jdx += 1
                        self.i_train.append(int(self.i_sample))
                    else:
                        self.i_train.append(int(i_max))

                coordinatePoint = self.x_true[self.i_train[-1]] + x_center
                self.testedWaypoints.append(list(coordinatePoint))
                coord = tuple(coordinatePoint.flatten())
                if coord in coordinate_to_number:
                    self.flag = coordinate_to_number[coord]

                print("Rover Moving to Location --> Flag: " + str(self.flag) + " & Coordinate: " + str(coord))
                print("Sample Status: " + str(self.samp) + "/" + str(self.sample_iter) + " Total Samples")
                self.update_visualization(list(coordinatePoint))

                cont = True
                while cont:
                    print("[T-Minus 20 sec until data is collected]")
                    time.sleep(10)  # Wait 20 seconds for rover to move to position & insert probe
                    print("[T-Minus 10 sec until data is collected]")
                    time.sleep(5)
                    print("[T-Minus 5 sec until data is collected]")
                    time.sleep(5)
                    print("Collecting Data... [T-Minus 100 sec until completion]")
                    cont, data_average = self.process_data()

                print("Average Moisture Percentage:", data_average)
                actual_moisture = float(data_average)
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                y_obs = np.append(y_obs, float(data_average))
                file_path = os.path.join(self.new_folder_path, "GPAL_tested_coordinates.txt")
                testedCoord = open(file_path, "a")
                testedCoord.write(str(coordinatePoint) + '\n')
                testedCoord.close()
                file_path = os.path.join(self.new_folder_path, "GPAL_correlation_values.txt")
                moistVals = open(file_path, "a")
                moistVals.write(str(data_average) + '\n')
                moistVals.close()
                self.count += 1
                self.iteration += 1
                self.samp += 1

                self.retroactive_error.append(np.linalg.norm(y_pred_next - actual_moisture))

                self.fig2 = plt.figure()
                self.plotGP()
                self.fig2.tight_layout()
                self.fig2.savefig(self.new_folder_path + '/' + str(len(set(self.i_train))) + '.png')
                self.fig2.clear()
                plt.close(self.fig2)

            except Exception as e:
                print("An exception occurred:", e)

        ###############################################################################
        ################################## Save Data ##################################
        ###############################################################################
        # Convert images to video
        video_name = self.image_path + '_GPAL_' + self.TrialName + '.mp4'

        images = []
        int_list = []
        for img in os.listdir(self.image_path):
            if img.endswith(".png"):
                images.append(img)
                s = re.findall(r'\d+', img)
                try:
                    if s:  # Check if any digits were found
                        int_list.append(int(s[0]))
                    else:
                        print("No digits found for filename:", img)
                except ValueError:
                    print("Non-integer digits found for filename:", img)

        arg_list = np.argsort(int_list)

        frame = cv2.imread(os.path.join(self.image_path, images[0]))
        height, width, layers = frame.shape

        video = cv2.VideoWriter(
            video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (width, height))

        for i in range(len(arg_list)):
            image = images[arg_list[i]]
            video.write(cv2.imread(os.path.join(self.image_path, image)))

        cv2.destroyAllWindows()
        video.release()

        class Data:
            def __init__(self, x_true, y_obs, i_train, var_iter_global,
                         var_iter_local, x_test_local, x_test_global, covar_global, covar_trace,
                         covar_totelements, covar_nonzeroelements, f2_H_local, f2_H_global):
                self.x_true = x_true
                self.y_obs = y_obs
                self.i_train = i_train
                self.var_iter_global = var_iter_global
                self.var_iter_local = var_iter_local
                self.x_test_local = x_test_local
                self.x_test_global = x_test_global
                self.covar_global = covar_global
                self.covar_trace = covar_trace
                self.covar_totelements = covar_totelements
                self.covar_nonzeroelements = covar_nonzeroelements
                self.f2_H_local = f2_H_local
                self.f2_H_global = f2_H_global

        mydata = Data(self.x_true, y_obs, self.i_train, self.var_iter_global,
                      self.var_iter_local, self.x_test_local, self.x_test_global,
                      self.covar_global, self.covar_trace, self.covar_totelements, self.covar_nonzeroelements,
                      self.f2_H_local, self.f2_H_global)

        # Save data to a pickle file
        file_path = os.path.join(self.new_folder_path, "saved_data.pkl")
        with open(file_path, 'wb') as file:
            pickle.dump(mydata, file)

        if self.MOVEFILES:
            self.move_txt_files(os.path.expanduser("~/field_testing_al/" + self.TrialName + str(self.TrialNumber)), self.new_folder_path)

def main(args=None):
    rclpy.init(args=args)
    node = FieldTestNode()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
