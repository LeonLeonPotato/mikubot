import torch

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

lookback = 16

lstm_layers = 1
lstm_hidden_size = 16
present_hidden_size = 64

split_size = 0.1
batch_size = 256
learning_rate = 0.0005

save_name = 'model.pth'