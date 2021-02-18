# **Traffic Sign Recognition** 

## Project scope

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report

[//]: # (Image References)

[image1]: ./examples/preview.png "Preview"
[image2]: ./examples/distribution.png "Distribution"
[image3]: ./examples/augmentation.png "Augmentation"
[image4]: ./examples/web_prediction.png "Web prediction"
[image5]: ./examples/web_top5_1.png "Web top 5"
[image6]: ./examples/web_top5_2.png "Web top 5"
[image7]: ./examples/web_top5_3.png "Web top 5"
[image8]: ./examples/web_top5_4.png "Web top 5"
[image9]: ./examples/grad_cam_1.png "Grad-CAM"
[image10]: ./examples/grad_cam_2.png "Grad-CAM"
[image11]: ./examples/grad_cam_3.png "Grad-CAM"
[image12]: ./examples/grad_cam_4.png "Grad-CAM"

---
### Writeup / README

The entire project was developed in [this notebook](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb). 
The steps are described below:

### Data Set Summary & Exploration

I used the Pandas library to calculate summary statistics of the traffic
signs data set:

* The size of training set is `34799`
* The size of the validation set is `4410`
* The size of test set is `12630`
* The shape of a traffic sign image is `(32, 32)`
* The number of unique classes/labels in the data set is `43`

I made a simple analysis of the dataset. First I previewed random samples of images and labels from all sets:

![preview][image1]

Another thing of interest was the distribution of samples per class.
I am interested to see if there are some very under-represented classes and if the distributions match.
The barplot shows the distributions are pretty similar, which is evident also in the final results of the model evaluation.
![distribution][image2]

### Design and Test a Model Architecture

I have started with implementing the LeNet-5 architecture from the classroom.
But since I am more familiar with the Keras api of Tensorflow, I wanted to try a [new implementation](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Model-Architecture), 
not simply reusing the code from the course.

I started with normalizing the data with the idea of adding other transformations later 
(like grayscale conversion or data augmentation). For this reason I experimented with defining the pre-processing
as part of the model itself.

For normalization I used an experimental layer from keras that I first fitted on the training data to learn the dataset mean
and variance, then this values get applied inside the model itself. 
Just with this plain model I got to an accuracy 0.9936 on the train and 0.9141 on the validation dataset.

From this on, I started with adding various changes to see if I can further improve the model, and combat over/under-fitting.

|Change                                   |Train Accuracy|Validation Accuracy|Remarks        |
|-----------------------------------------|--------------|-------------------|---------------|
|Vanilla model                            |        0.9963|             0.9141|               |
|2 X Dropout 0.5                          |        0.9685|             0.9583|reduced overfit|
|Convert to grayscale                     |        0.9690|             0.9615|color doesn't seem to be very important|
|Augmentation: brightness, zoom, rotation |        0.9685|             0.9583|validation accuracy more stable at the end of training|
|Augmentation: image shift                |        0.8424|             0.9161| does not help |
|Introduce learning schedule              |        0.9607|             0.9741|               |

The best results I saw for reducing the overfitting were from [image augmentation](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Image-augmentation) and dropout, so I kept both.
One thing to note is that not all augmentations were useful. For example, shifting the image made the model worse.
I assume this is because all signs are center cropped, so shifting them actually confused the model.

Below are some examples of augmented images:
![augmentation][image3]

The final model's architecture looks like this:

```properties
__________________________________________________________________________________________________
Layer (type)                    Output Shape         Param #     Connected to                     
==================================================================================================
input_3 (InputLayer)            [(None, 32, 32, 3)]  0                                            
__________________________________________________________________________________________________
tf_op_layer_Identity_4 (TensorF [(None, 32, 32, 3)]  0           input_3[0][0]                    
__________________________________________________________________________________________________
tf_op_layer_Shape_2 (TensorFlow [(4,)]               0           tf_op_layer_Identity_4[0][0]     
__________________________________________________________________________________________________
tf_op_layer_GatherV2_4 (TensorF [(3,)]               0           tf_op_layer_Shape_2[0][0]        
__________________________________________________________________________________________________
tf_op_layer_GatherV2_5 (TensorF [(1,)]               0           tf_op_layer_Shape_2[0][0]        
__________________________________________________________________________________________________
tf_op_layer_Prod_4 (TensorFlowO [()]                 0           tf_op_layer_GatherV2_4[0][0]     
__________________________________________________________________________________________________
tf_op_layer_Prod_5 (TensorFlowO [()]                 0           tf_op_layer_GatherV2_5[0][0]     
__________________________________________________________________________________________________
tf_op_layer_Transpose_2 (Tensor [(None, 32, 32, 3)]  0           tf_op_layer_Identity_4[0][0]     
__________________________________________________________________________________________________
tf_op_layer_stack_2 (TensorFlow [(2,)]               0           tf_op_layer_Prod_4[0][0]         
                                                                 tf_op_layer_Prod_5[0][0]         
__________________________________________________________________________________________________
tf_op_layer_Reshape_4 (TensorFl [(None, None)]       0           tf_op_layer_Transpose_2[0][0]    
                                                                 tf_op_layer_stack_2[0][0]        
__________________________________________________________________________________________________
tf_op_layer_MatMul_2 (TensorFlo [(None, 1)]          0           tf_op_layer_Reshape_4[0][0]      
__________________________________________________________________________________________________
tf_op_layer_concat_2 (TensorFlo [(3,)]               0           tf_op_layer_GatherV2_4[0][0]     
__________________________________________________________________________________________________
tf_op_layer_Reshape_5 (TensorFl [(None, 32, 32)]     0           tf_op_layer_MatMul_2[0][0]       
                                                                 tf_op_layer_concat_2[0][0]       
__________________________________________________________________________________________________
tf_op_layer_ExpandDims_2 (Tenso [(None, 32, 32, 1)]  0           tf_op_layer_Reshape_5[0][0]      
__________________________________________________________________________________________________
tf_op_layer_Identity_5 (TensorF [(None, 32, 32, 1)]  0           tf_op_layer_ExpandDims_2[0][0]   
__________________________________________________________________________________________________
normalization (Normalization)   (None, 32, 32, 1)    3           tf_op_layer_Identity_5[0][0]     
__________________________________________________________________________________________________
conv2d_4 (Conv2D)               (None, 28, 28, 6)    156         normalization[2][0]              
__________________________________________________________________________________________________
max_pooling2d_4 (MaxPooling2D)  (None, 14, 14, 6)    0           conv2d_4[0][0]                   
__________________________________________________________________________________________________
conv2d_5 (Conv2D)               (None, 10, 10, 16)   2416        max_pooling2d_4[0][0]            
__________________________________________________________________________________________________
max_pooling2d_5 (MaxPooling2D)  (None, 5, 5, 16)     0           conv2d_5[0][0]                   
__________________________________________________________________________________________________
flatten_2 (Flatten)             (None, 400)          0           max_pooling2d_5[0][0]            
__________________________________________________________________________________________________
dense_6 (Dense)                 (None, 120)          48120       flatten_2[0][0]                  
__________________________________________________________________________________________________
dropout_4 (Dropout)             (None, 120)          0           dense_6[0][0]                    
__________________________________________________________________________________________________
dense_7 (Dense)                 (None, 84)           10164       dropout_4[0][0]                  
__________________________________________________________________________________________________
dropout_5 (Dropout)             (None, 84)           0           dense_7[0][0]                    
__________________________________________________________________________________________________
dense_8 (Dense)                 (None, 43)           3655        dropout_5[0][0]                  
==================================================================================================
Total params: 64,514
Trainable params: 64,511
Non-trainable params: 3
```
Because I wanted to introduce image pre-processing as part of the model, the internal keras implementation 
of grayscale conversion introduces 14 layers (with no learnable parameters), with the next layer performing the normalization.
The rest of the layers are an exact implementation of the LeNet architecture, with the addition of two Dropout layers.

The last improvement was to train a bit longer and to start [decreasing the learning rate](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Learning-rate-scheduling) as the training progressed.
I originally started experimenting with a small number of epochs (around 20) and a constant learning rate of 1e-3. 
Since the network kept learning, I kept increasing the number of epochs until about 70 (when I stopped seeing any improvement). 
After 50 epochs, when progress appeared to be halting I started to exponentially decrease the learning rate, 
so that I don't risk overshooting the found minima.  

Regarding the batch size, I experimented with different values but I saw no visible differences, so I used 128, 
small enough to fit into my GPU memory, but large enough to keep the GPU busy.

The end result is a training accuracy of 0.967 and a validation accuracy of 0.9741.
When computing the top-5 accuracy, the score is 0.9961 for train and 0.9966 for validation.

Once these results were good enough, I used the test data as well and obtained an accuracy of **0.9568** and a top-5
accuracy of **0.9939**.
Given that the test accuracy is smaller than the validation, but close to the training accuracy makes me think that
the validation dataset is too small. Given some more data augmentation, training for a longer time, 
and using a cross-validation technique might be a possible way to further improve the model's results. 

### Test a Model on New Images

I searched on the web for some other examples of german traffic signs to see how the model performs on images outside of the provided dataset.

I wanted to have a fully reproducible notebook, so I picked license-free images from Wikipedia, 
that are [automatically downloaded](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Load and Output the Images), 
[center-cropped and resized](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Image-pre-processing) to our desired resolution. 

The downside to this approach is that they are actually vector graphics and not photographs, and as such proved a too easy test for the model.

Here are the results of the [prediction](https://nbviewer.jupyter.org/github/rbarbantan/Udacity-CarND/blob/master/P3-Traffic-Sign-Classifier/Traffic_Sign_Classifier.ipynb#Predict-the-Sign-Type-for-Each-Image):
![web-prediction][image4]
Since the images have perfect positioning and brightness, the model scored a perfect 100% accuracy. 
This made me curious try and better understand what that is, to make sure that there is no bug in the code.
So for each image, I took the top-5 predicted classes and sampled images from the original dataset, for an easy visual exploration:

![web-top-5][image5]
![web-top-5][image6]
![web-top-5][image7]
![web-top-5][image8]

You can easily see that the model prefers classes with very similar shapes, and does not care much about the color.

The last thing I tried was to also get a glimpse inside the model itself, so I tried and approach caled 
*Gradient-weighted class activation mapping* (**Grad-CAM**). It basically tries to visualize the activations 
for each layer of the model, to what parts of the image most influence the prediction. 
Below are some results for the 60 km/h speed limit sign:

![grad-cam][image9]
For the first conolutional layer, I believe the activations are sensitive to edges, trying to find straight lines or curves.

![grad-cam][image10]
The next layer seems to focus on the numbers themselves, perhaps trying to differentiate between the 30, 60, etc speed limit signs.

![grad-cam][image11]
![grad-cam][image12]
I tried to get an intuition on the next layers as well, but I have to admit I didn't find any obvious pattern.

Given this extra analysis, I am confident that the model did indeed learn to successfully predict traffic signs,
and that having a 95.68 % accuracy on the test is quite a good result for such a small model.