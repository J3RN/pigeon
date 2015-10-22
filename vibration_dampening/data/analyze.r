acc_data <- read.csv("padding.csv")

x = 1:length(acc_data$x)

plot(x, acc_data$x, type="n")
lines(x, acc_data$x, type="l")
lines(x, acc_data$y, type="l")
