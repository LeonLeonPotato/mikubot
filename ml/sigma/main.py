import torch
import torch.nn as nn
from torchvision import datasets, transforms

transformations = transforms.Compose([
    transforms.Resize((8, 8)),
    transforms.ToTensor(),
    transforms.Lambda(lambda x: x > 0.3),
    transforms.Lambda(lambda x: x.float())
])
mnist = datasets.MNIST('data', download=True, transform=transformations)

i = 188
sample = mnist[i][0].squeeze().numpy()
for row in sample:
   l = ''.join(map(lambda x: '1' if x else '0', row))
   print(l, end='\\\n')

print("Correct:", mnist[i][1])

exit()

class Net(nn.Module):
    def __init__(self, *args, **kwargs):
        super(Net, self).__init__(*args, **kwargs)
        self.fc = nn.Linear(8*8, 10)
        self.softmax = nn.Softmax(dim=1)

    def forward(self, x):
        single = len(x.shape) == 2
        if single:
            x = x.unsqueeze(1)
        x = torch.flatten(x, 1)
        x = self.fc(x)
        x = self.softmax(x)
        if single:
            x = x.squeeze(1)
        return x

model = Net()
model = model.cuda()
loader = torch.utils.data.DataLoader(mnist, batch_size=32, shuffle=True)
optimizer = torch.optim.Adam(model.parameters(), lr=0.001, betas=(0.9, 0.999))
crit = nn.CrossEntropyLoss()

for i in range(20):

    average_loss = 0
    average_accuracy = 0

    for x, y in loader:
        x = x.squeeze(1).cuda()
        y = y.cuda()
        output = model(x)
        loss = crit(output, y)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        average_loss += loss.item()
        average_accuracy += (output.argmax(1) == y).float().mean().item()
    
    average_loss /= len(loader)
    average_accuracy /= len(loader)
    print(average_loss, average_accuracy)
    
weights = model.fc.weight.data.cpu().numpy()
biases = model.fc.bias.data.cpu().numpy()

with open('weights.txt', 'w') as f:
    f.write("Weights = {\n\t")
    for row in weights:
        f.write('{' + ', '.join(map(str, row)) + '},\n\t')
    f.write('\n}')

with open('biases.txt', 'w') as f:
    f.write("Biases = {\n\t")
    f.write(', '.join(map(str, biases)) + '\n\t')
    f.write('\n}')
