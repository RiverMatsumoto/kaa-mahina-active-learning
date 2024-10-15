#!/usr/bin/env python3

# /home/roselab/miniconda3/envs/ml_rover/bin/python3
# -*- coding: utf-8 -*- #
"""
ʕっ•ᴥ•ʔっ
@authors: jen & sapph

last updated: aug 19 2024 09:22
"""

#ros imports
import rclpy
from rclpy.node import Node

# basics
import os
#os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import scipy.stats as ss

# specialized
from numpy.linalg import inv
from osgeo import gdal
from PIL import Image
import threading
import torch
import gpytorch
import sklearn.metrics
import cv2

# other
from serial import Serial, SerialException
from scipy.stats import ks_2samp
from sklearn.metrics import mean_squared_error
import serial
#import pyautogui
import shutil
import pathlib
import glob
#import keyboard``
import datetime
import random
import csv
import pickle
import re
import math
import time
import sys

#moisture sensor usage
MOISTURESENSOR = True
#science blind vs. active learning
SNAKEPATTERN = False
SPIRALPATTERN = False
#other
MOVEFILES = False

#naming convention
TrialNumber = sys.argv[1]
TrialName = "AL_Trial"

#field test grid sizing [100x100 sqaure]
grid_length = 10
grid_width = 10

#ml parameters
kernel_type = sys.argv[2]
#kernel_type = 'RBF', 'Matern'
poly_rank = 4
r_disp = 6.0
constraint_flag = 1
local_flag = 1

if local_flag ==1:
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
############################# function definitions ############################
###############################################################################
class ExactGPModel(gpytorch.models.ExactGP):   # define the simplest form of GP model, exact inference 
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
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
        elif kernel_type == 'Cosine': # !
        # Cosine Kernel
            self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.CosineKernel())
        elif kernel_type == 'Linear':
        # Linear Kernel
            self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.LinearKernel())
        elif kernel_type == 'Polynomial':
        # Polynomial Kernel 
            self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.PolynomialKernel(ard_num_dims = 3, power = poly_rank))
        
    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

def RKHS_norm(y,sigma,K): # calculate RKHS norm from covariance
    n_row, n_col = K.shape
    alpha = inv(K + sigma**2 * np.eye(n_row)) 
    return alpha.transpose() 
     
def unique_sample(i_sample,i_set,i_train,i_max,x):
    if i_sample <= i_max and i_sample >= 0:
        if i_sample not in i_train:
            i_new = i_sample
        else:
            i_set_unique = set(i_set)-set(i_train)
            if not i_set_unique:
                return None
            i_set_unique = list(i_set_unique)
            x_start = x[i_sample,:]
            x_disp = np.sqrt((x[i_set_unique,0]-x_start[0])**2 + (x[i_set_unique,1]-x_start[1])**2)
            # disp_i = np.abs(np.array(i_set_unique)-np.array(i_sample))
            i_new =i_set_unique[np.argmin(x_disp)]
    elif i_sample > i_max:
        i_new = unique_sample(i_sample-1,i_set,i_train,i_max)
    else:
        i_new = unique_sample(i_sample+1,i_set,i_train,i_max)
    return i_new

def sample_disp_con(x,x_start,r_disp):
    # x_start = x[i_start,:]
    x_disp = np.sqrt((x[:,0]-x_start[0])**2 + (x[:,1]-x_start[1])**2)# + (x[:,2]-x_start[2])**2)
    i_con = np.argwhere(x_disp<=r_disp)
    i_con = np.sort(i_con)
    return list(i_con[:,0])

def get_valid_float_input(prompt):
    while True:
        try:
            #user_input = simpledialog.askfloat("Input", prompt)
            user_input = float(input(prompt))
            if user_input is not None and 0 <= user_input <= 1:
                return user_input
            else:
                print("Invalid input. Please enter a float between 0 and 1.")
        except ValueError:
            print("Invalid input. Please enter a valid floating-point number.")

def GPtrain(x_train, y_train, training_iter):
    
    # initialize likelihood and model
    likelihood = gpytorch.likelihoods.GaussianLikelihood()
    model = ExactGPModel(x_train, y_train, likelihood)
    # Find optimal model hyperparameters
    model.train()
    likelihood.train()
    
    # Use the adam optimizer
    optimizer = torch.optim.Adam([
        {'params': model.parameters()},  # Includes GaussianLikelihood parameters
    ], lr=0.1)
    
    # "Loss" for GPs - the marginal log likelihood
    mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)

    # train GP moel
    for i in range(training_iter):
        # Zero gradients from previous iteration
        optimizer.zero_grad()
        # Output from model
        output = model(x_train)
        # Calc loss and backprop gradients
        loss = -mll(output, y_train)
        loss.backward()
        optimizer.step()
    return likelihood, model, optimizer, output, loss

def GPeval(x_test, model, likelihood):
    
    # Get into evaluation (predictive posterior) mode
    model.eval()
    likelihood.eval()
    
    # Make predictions by feeding model through likelihood
    with torch.no_grad(), gpytorch.settings.fast_pred_var():
        observed_pred = likelihood(model(x_test))
        
    f_preds = model(x_test)
    y_preds = likelihood(model(x_test))
    f_mean = f_preds.mean
    f_var = f_preds.variance
    f_covar = f_preds.covariance_matrix
    
    with torch.no_grad():
        # Get upper and lower confidence bounds
        lower, upper = observed_pred.confidence_region()
        
    return observed_pred, lower, upper

def create_folder_with_suffix(folder_path):
    i = 1
    while True:
        new_folder_path = folder_path + f"_{i}" if i > 1 else folder_path
        if not os.path.exists(new_folder_path):
            os.makedirs(new_folder_path)
            return new_folder_path
        i += 1
        
def snake_pattern(grid):
    result = []
    for i, row in enumerate(grid):
        if i % 2 == 0:
            result.extend(row)
        else:
            result.extend(reversed(row))
    return result

def spiral_traversal(grid):
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

def update_visualization(next_point, sampled_points):
    sampled_points = sampled_points[:-1]
    grid = np.zeros((grid_length, grid_width)) 
    ax5 = fig1.add_subplot(1, 1, 1)
    ax5.imshow(grid, cmap='gray', extent=[-0.5, grid_width - 0.5, -0.5, grid_length - 0.5]) 
    ax5.plot(np.array(next_point)[0], np.array(next_point)[1], marker='o', color='green') 
    ax5.scatter(np.array(sampled_points)[:, 0], np.array(sampled_points)[:, 1], marker='x', color='red') 
    ax5.set_title('Rover Exploration')
    ax5.set_xlabel('X')
    ax5.set_ylabel('Y')
    ax5.set_xticks(np.arange(0, grid_width, 1))
    ax5.set_yticks(np.arange(0, grid_length, 1))
    ax5.grid(True)

    plt.pause(1)  # Pause to allow the plot to be displayed
    image_file_path = os.path.join(new_folder_path, "GPAL_Path.png")
    fig1.savefig(image_file_path)

    


def process_data():
    
    cont = False
    data_average = None

    ##################################################
    ########## Moisture Sensor Utilization ###########
    ##################################################
    if MOISTURESENSOR:
        try:
            ser = serial.Serial('COM5',9600)
            time.sleep(2)
        
            with open('moisture_data_trial'+str(TrialNumber)+'_flag'+str(flag)+'.csv', 'a') as file:
                file.write("Time,MoistureLevel\n") #header of csv file
                start_time = time.time()
                while time.time() - start_time <= 100: #CHANGE BACK TO 100 SEC
                    line = ser.readline().decode('utf-8').rstrip()
                    file.write(line + '\n')
            ser.close()
            print("Data collection complete. Processing data...")
        
            #load data from reading
            data = pd.read_csv('moisture_data_trial'+str(TrialNumber)+'_flag'+str(flag)+'.csv')
        
            #convert miliseconds to seconds
            moisture_data = data['MoistureLevel']
            #print(moisture_data)
            data_average_sum = sum(moisture_data)
            #print(data_average_sum)
            data_average_len = len(moisture_data)
            #print(data_average_len)
            data_average = data_average_sum/data_average_len
            #print(data_average)
                    
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
  
def calculate_model_drift(y_baseline, y_new):
    mse_drift = mean_squared_error(y_baseline, y_new)
    return mse_drift

def plotGP(drift_values, rpe):
    ax1 = fig2.add_subplot(2, 3, 1)
    fig2.figsize=(14, 10)

    rover_path = ax1.plot(x_true[i_train,0],x_true[i_train,1], color='black')
    LAMP = ax1.scatter(x_true[:,0], x_true[:,1], s=1, alpha = 0.25)
    ax1.set_title('Rover Path [Exploration]', fontsize=10)
    ax1.set_xlabel('X [km]', fontsize=8)
    ax1.set_ylabel('Y [km]', fontsize=8) 
    ax1.tick_params(axis='both', which='major', labelsize=8)
    ax1.tick_params(axis='both', which='minor', labelsize=8)

    ax2 = fig2.add_subplot(2, 3, 2, projection='3d')
    rover = ax2.scatter3D(x_true[i_sample, 0], x_true[i_sample, 1],
                          y_train[samp-1], s=100, color='black', marker='*', zorder=1)
    points_pred_surf = ax2.plot_trisurf(
        x_test_global[:, 0], x_test_global[:, 1], observed_pred_global.mean.numpy(), cmap='inferno', linewidth=0, alpha=0.25, vmax=max(y_train), vmin=min(y_train))
    # shade between the lower and upper confidence bounds
    for i_test in range(len(x_test_local)):
        ax2.plot(x_test_local[i_test, 0].numpy()*np.array([1, 1]), x_test_local[i_test, 1].numpy()*np.array(
            [1, 1]), np.array([lower_local[i_test].numpy(), upper_local[i_test].numpy()]), 'gray')
    ax2.view_init(20, 20)
    ax2.set_xlabel('X', fontsize=8)
    ax2.set_ylabel('Y', fontsize=8)
    ax2.set_zlabel('Percentage Moisture [%]', fontsize=8)
    ax2.tick_params(axis='z', labelsize=8)
    ax2.set_title('Moisture Distribution \n[Model Prediction]', fontsize=10)
    ax2.tick_params(axis='both', which='major', labelsize=8)
    ax2.tick_params(axis='both', which='minor', labelsize=8)
    
    ax3 = fig2.add_subplot(2, 3, 4)
    ax3.plot(range(0, len(var_iter_local)),var_iter_local, color='blue', marker='.')
    ax3.plot(range(0, len(var_iter_global)),var_iter_global, color='black', marker='*')
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
    ax5.plot(range(0, len(rpe)), rpe, label='Global', color='black')
    ax5.set_xlabel('Number of Samples', fontsize=8)
    ax5.set_ylabel('Error in Moisture Estimate', fontsize=8)
    ax5.set_title('Retroactive Prediction \nError', fontsize=10)
    
    return ax1, ax2, ax3, ax4, ax5

def move_txt_files(source_folder, destination_folder):
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

###############################################################################
############################# ROS Implementation ##############################
###############################################################################
class FieldTestNode(Node):
    def __init__(self):
        super().__init__('field_test_node')
        # Initialize your code here

    def run(self):
        
        # new_folder_path = create_folder_with_suffix(r"C:\Users\sapph\OneDrive\Documents\RoSE_Field_Testing\\" + TrialName + TrialNumber)
        new_folder_path = create_folder_with_suffix(r"~/field_testing_al/" + datetime.now().strftime('%Y-%m-%d_%H-%M-%S') + "/" + TrialName + TrialNumber)
        parent_dir = '/Users/sapph/OneDrive/Documents/RoSE_Field_Testing'
        image_path = parent_dir
        
        fig1 = plt.figure()
        
        ###############################################################################
        ######################### Defining space to be tested #########################
        ###############################################################################
        x_coordinates = np.arange(grid_length)
        y_coordinates = np.arange(grid_width)
        x_grid, y_grid = np.meshgrid(x_coordinates, y_coordinates)
        x_true = np.stack((x_grid, y_grid), axis=-1)
        x_true = x_true.tolist()
        x_true = [item for sublist in x_true for item in sublist]
        x_true = np.array(x_true)

        n = int(np.sqrt(len(x_true)))
        m = n
        grid = x_true.reshape((n, m, 2))

        num = 0
        coordinate_to_number = {}
        for i, row in enumerate(grid):
            for j, coord in enumerate(row):
                coordinate_to_number[tuple(coord)] = num
                num += 1

        # Data normalizaton/standardization
        x_center_all = np.mean(x_true,0)
        x_disp = np.sqrt((x_true[:,0]-x_center_all[0])**2 + (x_true[:,1]-x_center_all[1])**2)
        i_min = np.argmin(x_disp)
        x_center = x_true[i_min,:]
        x_true = x_true - x_center
        ###############################################################################
        ########################### Constants for Training ############################
        ###############################################################################
        n_samples = len(x_true)
        r_NN = np.sqrt(1)#-0.29 #np.sqrt(3)*0.25 
        r_con = r_NN
        i_seq = list(range(0,n_samples))

        #%% hyperparameters for exploration training
        if SNAKEPATTERN or SPIRALPATTERN:
            sample_iter = grid_length * grid_width
        else:
            sample_iter = int(n_samples/2)
        training_iter = 100
        i_train = []
        var_iter = []
        var_iter_local = []
        var_iter_global = []
        lengthscale = []

        covar_global = []
        covar_trace = []
        covar_totelements = []
        covar_nonzeroelements = []
        f2_H_global = []
        f2_H_local = []
        flags = []

        if kernel_type == 'RBF' or kernel_type == 'Matern'  or kernel_type == 'Piece_Polynomial':
            noise = []
        elif kernel_type == 'Periodic':
            period_length = []
        elif kernel_type == 'RQ':
            alpha = []
        elif kernel_type == 'Linear':
            variance = []
        elif kernel_type == 'Polynomial':
            offset = []

        ###############################################################################
        # randomly sample next 10 data points with a displacement constraint of 10int #
        ###############################################################################
        if SNAKEPATTERN:
            gridIndex = [[i + j * grid_length for i in range(grid_width)] for j in range(grid_length)]
            i_train_known = snake_pattern(gridIndex)
            i_train.append(i_train_known[0])
            i_sample = i_train_known[0]
        elif SPIRALPATTERN:
            gridIndex = [[i + j * grid_length for i in range(grid_width)] for j in range(grid_length)]
            i_train_known = spiral_traversal(gridIndex)
            i_train.append(i_train_known[0])
            i_sample = i_train_known[0]
        else:
            random.seed(None) #42
            i_0 = random.randrange(n_samples) #%% randomly initialize location
            i_train.append(i_0)
            for i in range(5): #5 total points
                i_sample_set = sample_disp_con(x_true,x_true[i_train[-1]],r_NN) # nearest neighbor (within 0.25 km)
                #i_sample_set = sample_disp_con(x_true,x_true[i_train[-1]],r_con) # within 1 km
                i_sample = i_sample_set[random.randrange(len(i_sample_set))]
                i_train.append(int(i_sample))
            i_train = list(set(i_train))
            print("Number of initial points:",len(i_train))

        #Creating first 10 payload input values corresponsing to above
        y_obs = np.array([])
        testedWaypoints = []
        count = 0
        iteration = 0

        for coordinate in x_true[i_train,:]:
            coordinatePoint = coordinate + x_center
            #print(coordinatePoint)
            testedWaypoints.append(coordinatePoint)
    
            coord = tuple(coordinatePoint.flatten())
            if coord in coordinate_to_number:
                flag = coordinate_to_number[coord]

            print("Rover Location --> Flag: "+str(flag)+" & Coordinate: "+str(coord))
            if count != 0:
                rover_path = [item.tolist() if isinstance(item, np.ndarray) else item for item in testedWaypoints]
                update_visualization(list(coordinatePoint), rover_path)
   
            cont = True
            while cont:
                print("[T-Minus 20 sec until data is collected]")
                time.sleep(10) #wait 20 seconds for rover to move to position & insert probe
                print("[T-Minus 10 sec until data is collected]")
                time.sleep(5)
                print("[T-Minus 5 sec until data is collected]")
                time.sleep(5)
                print("Collecting Data... [T-Minus 100 sec until completion]")
                cont, data_average = process_data()

            #user_input = get_valid_float_input("Please enter payload input: ")
            print("Average Moisture Percentage: ", data_average)
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            y_obs = np.append(y_obs, float(data_average))
            file_path = os.path.join(new_folder_path, "GPAL_tested_coordinates.txt")
            testedCoord = open(file_path, "a")
            testedCoord.write(str(coordinatePoint) + '\n')
            testedCoord.close()
            file_path = os.path.join(new_folder_path, "GPAL_correlation_values.txt")
            moistVals = open(file_path, "a")
            moistVals.write(str(data_average) + '\n')
            moistVals.close()
            count+=1
            iteration += 1
    
        #Baseline model for prediction & drift calculations
        x_train = torch.from_numpy(x_true[i_train, :]).float()
        y_train = torch.from_numpy(y_obs).float()

        likelihood, model, optimizer, output, loss = GPtrain(x_train, y_train, training_iter)

        retroactive_error = []
        drift_values = []
        min_drift = float('inf')
        max_drift = float('-inf')

        ##########
        samp = 1 #
        ###############################################################################
        ###### EXPLORATION PHASE: for loop to append sample and retrain GP model ######
        ###############################################################################
        for j in range(sample_iter):
            # define training data
            x_train = torch.from_numpy(x_true[i_train,:])
            y_train = torch.from_numpy(y_obs)

            x_train = x_train.float()
            y_train = y_train.float()

            likelihood, model, optimizer, output, loss = GPtrain(x_train, y_train, training_iter)
   
            # store optimal hyperparameters
            if kernel_type == 'RBF' or kernel_type == 'Matern' or kernel_type == 'Piece_Polynomial':
                noise.append(model.likelihood.noise.detach().numpy())
                lengthscale.append(model.covar_module.base_kernel.lengthscale.detach().numpy()[0])
            elif kernel_type == 'Periodic':
                period_length.append(model.covar_module.base_kernel.period_length.detach().numpy()[0])
                lengthscale.append(model.covar_module.base_kernel.lengthscale.detach().numpy()[0])
            elif kernel_type == 'RQ':
                alpha.append(model.covar_module.base_kernel.alpha.detach().numpy()[0])
                lengthscale.append(model.covar_module.base_kernel.lengthscale.detach().numpy()[0])
            elif kernel_type == 'Linear':
                variance.append(model.covar_module.base_kernel.variance.detach().numpy()[0])
            elif kernel_type == 'Polynomial':
                offset.append(model.covar_module.base_kernel.offset.detach().numpy()[0])
        
            # validation set from observed data
            validation_samples = min(4, len(i_train))  # adjust to 10 w a full set
            i_val = i_train[-validation_samples:]
            x_val = torch.from_numpy(x_true[i_val, :]).float()  # validation set

            observed_pred, _, _ = GPeval(x_val, model, likelihood)
            y_new = observed_pred.mean.detach().numpy()
            y_baseline = y_obs[-validation_samples:]
            drift = np.mean(np.abs(y_baseline - y_new))  # calculate model drift

            min_drift = min(min_drift, drift)
            max_drift = max(max_drift, drift)
            normalized_drift = (drift - min_drift) / (max_drift - min_drift) if max_drift > min_drift else 0
            drift_values.append(normalized_drift)

            # Test points are regularly spaced centered along the last index bounded by index displacement
            i_con = sample_disp_con(x_true,x_true[i_train[-1]],r_con)
            x_test_local = torch.from_numpy(x_true[i_con,:]) # x_test is constrained to motion displacement
            x_test_global = torch.from_numpy(x_true[i_seq,:]) # x_test is the entire dataset
    
            x_test_local = x_test_local.float()
            x_test_global = x_test_global.float()
    
            # Evaluate RMS for local
            observed_pred_local, lower_local, upper_local = GPeval(x_test_local, model, likelihood)
            with torch.no_grad():
                f_preds = model(x_test_local)
                y_preds = likelihood(model(x_test_local))
                f_mean = f_preds.mean
                f_var_local = f_preds.variance
                f_covar = f_preds.covariance_matrix
            var_iter_local.append(max(f_var_local.numpy()))  

            # and global
            observed_pred_global, lower_global, upper_global = GPeval(x_test_global, model, likelihood)
            with torch.no_grad():
                f_preds = model(x_test_global)
                y_preds = likelihood(model(x_test_global))
                f_mean = f_preds.mean
                f_var_global = f_preds.variance
                f_covar = f_preds.covariance_matrix
            var_iter_global.append(max(f_var_global.numpy()))

            # evaluate covariance properties
            covar_global.append(f_covar)
            covar_trace.append(np.trace(f_covar.detach().numpy()))
            covar_totelements.append(np.size(f_covar.detach().numpy()))
            covar_nonzeroelements.append(np.count_nonzero(f_covar.detach().numpy()))
            mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood, model)
    
            sigma = 0.000289305 
            # and finally evaluate RKHS norm
            K_global = output._covar.detach().numpy()
            y_global = y_train.numpy().reshape(len(y_train),1)
            f2_H_sample = RKHS_norm(y_global,sigma,K_global)
            f2_H_global.append(f2_H_sample[0,0])
    
            n_set = len(i_train)
            n_sub = math.floor(n_set/2)
            i_sub = random.sample(range(1,n_set),n_sub)
            i_sub.sort()
            K_local = K_global[np.ix_(i_sub,i_sub)]
            y_local = y_global[i_sub]
            f2_H_sample = RKHS_norm(y_local,sigma,K_local)
            if j == 1 and SNAKEPATTERN and SPIRALPATTERN:
                f2_H_local.append(f2_H_sample[0,0])
    
            # pick the next point to sample by maximizing local variance and minimizing distance by sampling nearest neighbor along the way
            try: 
                if SNAKEPATTERN or SPIRALPATTERN:
                    # waypoint within r_con with maximum variance, nearest neighbor along the way
                    uncertainty = upper_local-lower_local
                    i_max = np.argmax(uncertainty)
                    x_max = x_test_local[i_max,:].numpy()
                    i_NN = sample_disp_con(x_true,x_true[i_train[-1]],r_NN)
                    dx_NN = np.sqrt((x_true[i_NN,0]-x_max[0])**2 + (x_true[i_NN,1]-x_max[1])**2)
                    i_dx = np.argsort(dx_NN)
                    i_train.append(i_train_known[j+1])
                    i_sample = i_train_known[j+1]
                elif local_flag ==1 or local_flag ==2:
                    # waypoint within r_con with maximum variance, nearest neighbor along the way
                    uncertainty = upper_local-lower_local
                    i_max = np.argmax(uncertainty)
                    x_max = x_test_local[i_max,:].numpy()
                    next_point = x_true[i_max]
                    observed_pred_next, _, _ = GPeval(torch.tensor([next_point]).float(), model, likelihood)
                    y_pred_next = observed_pred_next.mean.detach().numpy()[0]
                    i_NN = sample_disp_con(x_true,x_true[i_train[-1]],r_NN)
                    dx_NN = np.sqrt((x_true[i_NN,0]-x_max[0])**2 + (x_true[i_NN,1]-x_max[1])**2)
                    i_dx = np.argsort(dx_NN)
                    i_sample = []
                    j = 0
                    while not np.array(i_sample).size:
                        i_sample = unique_sample(i_NN[i_dx[j]],i_con,i_train,n_samples-1,x_true)
                        count1 = 0
                        while i_sample is None:
                            count1+=1
                            i_sample_set = sample_disp_con(x_true,x_true[i_train[-1]],np.sqrt(count1)-0.29)
                            i_sample = i_sample_set[random.randrange(len(i_sample_set))]
                            if i_sample in i_train:
                                i_sample = None
                            print("**OH NO: NON-UNIQUE POINT**")
                        j = j+1
                    i_train.append(int(i_sample))
                else:
                    # waypoint within global space with maximum variance, directly go
                    uncertainty = upper_global-lower_global
                    i_max = np.argmax(uncertainty)
                    # waypoint within entire space with max variance, nearest neighbor
                    if constraint_flag == 1:
                        a = x_test_global[i_max,:].numpy()
                        i_NN = sample_disp_con(x_true,x_true[i_train[-1]],r_NN)
                        dx_NN = np.sqrt((x_true[i_NN,0]-x_max[0])**2 + (x_true[i_NN,1]-x_max[1])**2)
                        i_dx = np.argsort(dx_NN)
                        # finds the nearest neigbor along the way to the point of highest variance
                        i_sample = []
                        j = 0
                        while not np.array(i_sample).size:
                            i_sample = unique_sample(i_NN[i_dx[j]],i_con,i_train,n_samples-1,x_true)
                            j = j+1
                        i_train.append(int(i_sample))
                    else:
                        i_train.append(int(i_max))

                coordinatePoint = x_true[i_train[-1]] + x_center
                testedWaypoints.append(list(coordinatePoint))
                coord = tuple(coordinatePoint.flatten())
                if coord in coordinate_to_number:
                    flag = coordinate_to_number[coord]

                print("Rover Moving to Location --> Flag: "+str(flag)+" & Coordinate: "+str(coord))
                print("Sample Status: "+str(samp)+"/"+str(sample_iter)+" Total Samples")
                rover_path = [item.tolist() if isinstance(item, np.ndarray) else item for item in testedWaypoints]
                update_visualization(list(coordinatePoint), rover_path)

                cont = True
                while cont:
                    print("[T-Minus 20 sec until data is collected]")
                    time.sleep(10) #wait 20 seconds for rover to move to position & insert probe
                    print("[T-Minus 10 sec until data is collected]")
                    time.sleep(5)
                    print("[T-Minus 5 sec until data is collected]")
                    time.sleep(5)
                    print("Collecting Data... [T-Minus 100 sec until completion]")
                    cont, data_average = process_data()
            
                #user_input = get_valid_float_input("Please enter payload input: ")
                print("Average Moisture Percentage:", data_average)
                actual_moisture = float(data_average)
                #print("Drift:", drift_values)
                print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                y_obs = np.append(y_obs, float(data_average))
                file_path = os.path.join(new_folder_path, "GPAL_tested_coordinates.txt")
                testedCoord = open(file_path, "a")
                testedCoord.write(str(coordinatePoint) + '\n')
                testedCoord.close()
                file_path = os.path.join(new_folder_path, "GPAL_correlation_values.txt")
                moistVals = open(file_path, "a")
                moistVals.write(str(data_average) + '\n')
                moistVals.close()
                count+=1
                iteration += 1
                samp += 1
        
                retroactive_error.append(np.linalg.norm(y_pred_next - actual_moisture))        

                fig2 = plt.figure()
                ax1, ax2, ax3, ax4, ax5 = plotGP(drift_values, retroactive_error)
                fig2.tight_layout()
                fig2.savefig(str(len(set(i_train)))+'.png')
                fig2.clear()
                plt.close(fig2)

            except Exception as e:
                print("An exception occurred:", e)

        ###############################################################################
        ################################## save data ##################################
        ###############################################################################

        #%% convert images to video
        video_name = image_path + '_GPAL_'+TrialName+'.mp4'

        images = []
        int_list = []
        for img in os.listdir(image_path):
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

        frame = cv2.imread(os.path.join(image_path, images[0]))
        height, width, layers = frame.shape

        video = cv2.VideoWriter(
            video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (width, height))

        for i in range(len(arg_list)):
            image = images[arg_list[i]]
            video.write(cv2.imread(os.path.join(image_path, image)))

        cv2.destroyAllWindows()
        video.release()

        class data:
            def __init__(self, x_true, y_obs, i_train, var_iter_global,
                         var_iter_local, x_test_local, x_test_global, covar_global, covar_trace, 
                         covar_totelements, covar_nonzeroelements, f2_H_local, f2_H_global):
                # likelihood, model, optimizer, output, loss,
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

        mydata = data(x_true, y_obs, i_train, var_iter_global, 
                      var_iter_local, x_test_local, x_test_global,
                      covar_global, covar_trace, covar_totelements, covar_nonzeroelements, f2_H_local, f2_H_global)
        x_true = mydata.x_true
        y_obs = mydata.y_obs
        i_train = mydata.i_train
        var_iter_global = mydata.var_iter_global
        var_iter_local = mydata.var_iter_local
        x_test_local = mydata.x_test_local
        x_test_global = mydata.x_test_global
        covar_global = mydata.covar_global
        covar_trace = mydata.covar_trace
        covar_totelements = mydata.covar_totelements
        covar_nonzeroelements = mydata.covar_nonzeroelements
        f2_H_local = mydata.f2_H_local
        f2_H_global = mydata.f2_H_global

        # Save data to a pickle file
        file_path = os.path.join(new_folder_path, "saved_data.pkl")
        with open(file_path, 'wb') as file:
            pickle.dump(mydata, file)

        move_txt_files(r"C:\Users\sapph\OneDrive\Documents\RoSE_Field_Testing\\" + TrialName + TrialNumber, new_folder_path)

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