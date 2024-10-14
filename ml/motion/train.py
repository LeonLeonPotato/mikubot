import torch as th
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt
import torch.utils.data as data
import models
import analyze
import constants

class WeightedL1Loss(nn.Module):
    def __init__(self, weight=None, reduction='mean'):
        super(WeightedL1Loss, self).__init__()
        self.weight = weight
        self.reduction = reduction

    def forward(self, inputs, targets):
        l1_loss = th.abs(inputs - targets)
        
        if self.weight is not None:
            l1_loss = l1_loss * self.weight
        
        if self.reduction == 'mean':
            return l1_loss.mean()
        elif self.reduction == 'sum':
            return l1_loss.sum()
        else:
            return l1_loss

if __name__ == "__main__":
    df = analyze.load_processed_df("driving_logs_1.csv")
    X = th.tensor(df.values).float()

    train = X[:int(len(X) * 0.8)]
    test = X[int(len(X) * 0.8):]

    train_dataset = analyze.RobotDataset(train, df)
    eval_dataset = analyze.RobotDataset(test, df)
    train_loader = data.DataLoader(train_dataset, batch_size=constants.batch_size, shuffle=True)
    eval_loader = data.DataLoader(eval_dataset, batch_size=constants.batch_size, shuffle=True)

    past_size = train_dataset[0][0].shape[-1]
    present_size = train_dataset[0][1].shape[-1]
    future_size = train_dataset[0][2].shape[-1]

    model = models.MotionModel(past_size, present_size, future_size).to(constants.device)
    criterion = WeightedL1Loss(weight=th.tensor([0, 0, 0, 0, 1, 1, 0, 0]).to(constants.device))
    optimizer = th.optim.AdamW(model.parameters(), lr=constants.learning_rate)

    def do_eval():
        model.eval()
        with th.inference_mode():
            total_loss = 0
            for past, present, future in eval_loader:
                past, present, future = past.to(constants.device), present.to(constants.device), future.to(constants.device)
                output = model(past, present)
                loss = criterion(output, future)
                total_loss += loss.item()
        model.train()
        return total_loss / len(eval_loader)

    global_steps = 0

    train_plot_x = []
    train_plot_y = []
    eval_plot_x = []
    eval_plot_y = []

    try:
        for epoch in range(1000):
            for past, present, future in train_loader:
                past, present, future = past.to(constants.device), present.to(constants.device), future.to(constants.device)

                optimizer.zero_grad()
                output = model(past, present)
                loss = criterion(output, future)

                loss.backward()
                optimizer.step()
                
                train_plot_x.append(global_steps)
                train_plot_y.append(loss.item())

                if global_steps % 50 == 0:
                    eval_loss = do_eval()

                    eval_plot_x.append(global_steps)
                    eval_plot_y.append(eval_loss)

                    print(f"step {global_steps} eval loss: {eval_loss} train loss: {loss.item()}")

                global_steps += 1
    except KeyboardInterrupt:
        print("Exiting early")

    th.save(model.state_dict(), f"ml/motion/{constants.save_name}")

    plt.plot(train_plot_x, train_plot_y, label="train")
    plt.plot(eval_plot_x, eval_plot_y, label="eval")
    plt.legend()
    plt.grid()
    plt.xlabel("Steps")
    plt.ylabel("Loss")
    plt.title("Training Loss vs global training steps")
    plt.show()
    
    