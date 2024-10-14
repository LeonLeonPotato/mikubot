import torch as th
import torch.nn as nn
import torch.nn.functional as F

from constants import *

class FutureModel(nn.Module):
    def __init__(self, input_size, output_size, psize=present_hidden_size):
        super(FutureModel, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, psize),
            nn.LeakyReLU(),
            nn.Linear(psize, psize),
            nn.LeakyReLU(),
            nn.Linear(psize, output_size)
        )
    
    def forward(self, x):
        return self.fc(x)

class MotionModel(nn.Module):
    def __init__(self, past_size, present_size, future_size, hs=lstm_hidden_size, layers=lstm_layers, psize=present_hidden_size):
        super(MotionModel, self).__init__()
        self.past_encoder = nn.LSTM(past_size, hs, layers, batch_first=True)
        self.future_processor = FutureModel(hs + present_size, future_size, psize=psize)

    def forward(self, past, present):
        if len(past.shape) == 2:
            past = past.unsqueeze(0)
        
        if len(present.shape) == 1:
            present = present.unsqueeze(0)

        past = self.past_encoder(past)[0][:, -1, :]
        future = self.future_processor(th.cat([past, present], dim=-1))
        return future