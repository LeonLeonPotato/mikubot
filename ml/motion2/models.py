import torch as th
import torch.nn as nn
import torch.nn.functional as F

from constants import *

class FutureModel(nn.Module):
    def __init__(self, input_size, output_size):
        super(FutureModel, self).__init__()
        self.fc = nn.Sequential(
            nn.Linear(input_size, present_hidden_size),
            nn.LeakyReLU(),
            nn.Linear(present_hidden_size, output_size)
        )
    
    def forward(self, x):
        return self.fc(x)

class MotionModel(nn.Module):
    def __init__(self, past_size, present_size, future_size):
        super(MotionModel, self).__init__()
        self.past_encoder = nn.LSTM(past_size, lstm_hidden_size, lstm_layers, batch_first=True)
        self.future_processor = FutureModel(lstm_hidden_size + present_size, future_size)

    def forward(self, past, present):
        if len(past.shape) == 2:
            past = past.unsqueeze(0)
        
        if len(present.shape) == 1:
            present = present.unsqueeze(0)

        past = self.past_encoder(past)[0][:, -1, :]
        future = self.future_processor(th.cat([past, present], dim=-1))
        return future