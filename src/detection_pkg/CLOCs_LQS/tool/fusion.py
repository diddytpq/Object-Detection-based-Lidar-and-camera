import torch
from torch import nn
from torch.nn import functional as F
from torch.nn import Sequential 


import sys
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

class fusion(nn.Module):
    def __init__(self):
        super(fusion, self).__init__()
        self.name = 'fusion_layer'
        self.corner_points_feature = Sequential(
            nn.Conv2d(24,48,1),
            nn.ReLU(),
            nn.Conv2d(48,96,1),
            nn.ReLU(),
            nn.Conv2d(96,96,1),
            nn.ReLU(),
            nn.Conv2d(96,4,1),
        )
        self.fuse_2d_3d = Sequential(
            nn.Conv2d(5,18,1),
            nn.ReLU(),
            nn.Conv2d(18,36,1),
            nn.ReLU(),
            nn.Conv2d(36,36,1),
            nn.ReLU(),
            nn.Conv2d(36,1,1),
        )
        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )



    def forward(self,input_1,tensor_index):
        flag = -1
        if tensor_index[0,0] == -1:
            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            flag = 0
        else:

            x = self.fuse_2d_3d(input_1)
            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            out_1[:,tensor_index[:,0],tensor_index[:,1]] = x[0,:,0,:]
            flag = 1
        x = self.maxpool(out_1)
        #x, _ = torch.max(out_1,1)
        x = x.squeeze().reshape(1,-1,1)
        return x,flag

class _1(nn.Module):
    def __init__(self):
        super(custom_fusion, self).__init__()
        self.name = 'fusion_layer'

        self.fuse_2d_3d = Sequential(
            nn.Conv2d(4,16,1),
            nn.BatchNorm2d(16, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            nn.Conv2d(16,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            nn.Conv2d(32,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            nn.Conv2d(64,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            nn.Conv2d(128,36,1),
            nn.BatchNorm2d(36, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            nn.Conv2d(36,1,1),
        )
        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )


class _2(nn.Module):
    def __init__(self):
        super(custom_fusion, self).__init__()
        self.name = 'fusion_layer'
        
        self.fuse_2d_3d = Sequential(
            nn.Conv2d(4,16,1),
            nn.BatchNorm2d(16, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            nn.Conv2d(16,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            nn.Conv2d(32,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            nn.Conv2d(64,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            nn.Conv2d(128,36,1),
            nn.BatchNorm2d(36, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            nn.Conv2d(36,1,1),

        )
        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )

class _3(nn.Module):
    def __init__(self):
        super(custom_fusion, self).__init__()
        self.name = 'fusion_layer'
        
        self.block_1 = Sequential(
            nn.Conv2d(5,16,1),
            nn.BatchNorm2d(16, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(16,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(32,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
        )

        self.block_2 = Sequential(
            nn.Conv2d(64,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(128,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
        )

        self.block_3 = Sequential(
            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            
            nn.Conv2d(64,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(32,1,1),

        )


        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )

class _4(nn.Module): #_4
    def __init__(self):
        super(custom_fusion, self).__init__()
        self.name = 'fusion_layer'

        self.block_1 = Sequential(
            nn.Conv2d(5,16,1),
            nn.BatchNorm2d(16, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),

            nn.Conv2d(16,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),

            nn.Conv2d(32,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
        )

        self.block_2 = Sequential(
            nn.Conv2d(64,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),

            nn.Conv2d(128,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),

            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
        )

        self.block_3 = Sequential(
            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),
            
            nn.Conv2d(64,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.LeakyReLU(),

            nn.Conv2d(32,1,1),

        )

        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )


    def forward(self,input_1,tensor_index):
        flag = -1
        if tensor_index[0,0] == -1:
            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            flag = 0
        else:

            x_1 = self.block_1(input_1)
            x_2 = self.block_2(x_1)

            x_12 = torch.cat((x_1,x_2),1)
            x = self.block_3(x_12)

            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            out_1[:,tensor_index[:,0],tensor_index[:,1]] = x[0,:,0,:]
            flag = 1
        x = self.maxpool(out_1)
        #x, _ = torch.max(out_1,1)
        x = x.squeeze().reshape(1,-1,1)
        return x,flag


class custom_fusion(nn.Module): #_5
    def __init__(self):
        super(custom_fusion, self).__init__()
        self.name = 'fusion_layer'
        
        self.block_1 = Sequential(
            nn.Conv2d(5,16,1),
            nn.BatchNorm2d(16, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(16,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(32,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
        )

        self.block_2 = Sequential(
            nn.Conv2d(64,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(128,128,1),
            nn.BatchNorm2d(128, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
        )

        self.block_3 = Sequential(
            nn.Conv2d(128,64,1),
            nn.BatchNorm2d(64, momentum=0.99, eps=1e-3),
            nn.ReLU(),
            
            nn.Conv2d(64,32,1),
            nn.BatchNorm2d(32, momentum=0.99, eps=1e-3),
            nn.ReLU(),

            nn.Conv2d(32,1,1),

        )


        self.maxpool = Sequential(
            nn.MaxPool2d([200,1],1),
        )


    def forward(self,input_1,tensor_index):
        flag = -1
        if tensor_index[0,0] == -1:
            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            flag = 0
        else:

            x_1 = self.block_1(input_1)
            x_2 = self.block_2(x_1)

            x_12 = torch.cat((x_1,x_2),1)
            x = self.block_3(x_12)

            out_1 = torch.zeros(1,200,20000,dtype = input_1.dtype,device = input_1.device)
            out_1[:,:,:] = -9999999
            out_1[:,tensor_index[:,0],tensor_index[:,1]] = x[0,:,0,:]
            flag = 1
        x = self.maxpool(out_1)
        #x, _ = torch.max(out_1,1)
        x = x.squeeze().reshape(1,-1,1)
        return x,flag
