import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

lookback = 8

lstm_layers = 1
lstm_hidden_size = 32
present_hidden_size = 16

split_size = 0.2
batch_size = 64
learning_rate = 0.0005

save_name = 'model.pth'