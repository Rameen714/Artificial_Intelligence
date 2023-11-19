# ---------------------------------------------Car Number Plate Recognition System for Punjab using ANN-----------------------------------------------------

# This is a program to identify Punjab License Plate of a Car using Artificial Neural Network
# This program take an Image input, recognizes the charcters on the number plate.
# and then it will give you the license plate number in text format.
import os
import numpy as np
import pandas as pd
import cv2
import imutils
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import math
import tensorflow as tf
from tensorflow import keras
import keras.backend as K
from keras import optimizers

from sklearn.metrics import f1_score
from keras.layers import Flatten, Dense, Conv2D, MaxPooling2D, Input, Dropout
from keras.models import Model, Sequential
from keras.preprocessing.image import ImageDataGenerator
from keras.optimizers import Adam

# Step 1: Collecting/ Finding Data Set 
# done

# Step 2: Cleaning Data Set 
# not required

# Step 3: Dividing Data Set into training, validation and testing
# starting

image = cv2.imread(r'D:\Users\Rameen-Laptop\Desktop\AI\Git\NumberPlateDataSet\Cars\DSC_0968.JPG')
plt.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
plt.title("Original Image")
plt.show()


# image resized and graph sub plotted
image = imutils.resize(image,width=500)
fig,ax = plt.subplots(2,2,figsize=(10,7))

# display original image
img = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
ax[0,0].imshow(img)
ax[0,0].set_title('Original Image')

# RGB to grayscale conversion
gray_img = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
ax[0,1].imshow(gray_img,cmap='gray')
ax[0,1].set_title('GrayScaled Image')

# applying bilateral filter on grayscaled image
gray_img = cv2.bilateralFilter(gray_img,11,17,17)
ax[1,0].imshow(gray_img,cmap='gray')
ax[1,0].set_title('Bilateral Filter')

# applying canny edges on bilateral filtered image
edged = cv2.Canny(gray_img,170,200)
ax[1,1].imshow(edged,cmap='gray')
ax[1,1].set_title('Canny Edges on filtered')


fig.tight_layout()
plt.show()

# Applying GaussianBlur for additional smoothing
gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
plt.imshow(gray_img,cmap='gray')
plt.title('Gaussian Blured Image')
plt.show()

# Applying adaptive thresholding
thresh = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
plt.imshow(thresh, cmap='gray')
plt.title('Adaptive Thresholding')
plt.show()


# Finding contours in the thresholded image
cnts = cv2.findContours(thresh.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]



#cnts = cv2.findContours(edged.copy(),cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
cnts = sorted(cnts,key= cv2.contourArea,reverse=True)[:30] #sort contours based on their minimum required area
print("Number of contours:", len(cnts))

NumberPlateCnt = None #currently we have no number plate contour

ROI = 0
count = 0
for c in cnts:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c,0.02 * peri, True)
    if len(approx) == 4: #slect the contour with 4 corners
        NumberPlateCnt = approx #this is our approximate numebr plate contour
        x,y,w,h = cv2.boundingRect(c)
        ROI = img[y:y+h, x:x+w]
        break

if NumberPlateCnt is not None:
    # Drawing the slected contour on teh original iamge
    cv2.drawContours(image, [NumberPlateCnt],-1,(0,255,0),2)

plt.imshow(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
plt.title("Detected License Plate")
plt.show()

plt.imshow(ROI)
plt.title("Extracted License Plate")
plt.show()

# Step 4: Building Model
# Step 5: Model Training
# Step 6: Model Evaluation (implementing matrics to evaluate model performance)