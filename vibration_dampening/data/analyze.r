acc_data <- read.csv("1.csv")

x = 1:length(acc_data$X)

# Set up a 2x2 grid of graphs
par(mfrow = c(2, 2))

# Plot X data
plot(x, acc_data$X, type="n")
lines(x, acc_data$X, type="l")

# Plot Y data
plot(x, acc_data$Y, type="n")
lines(x, acc_data$Y, type="l")

# Spectral analysis for X
spectrum(acc_data$X, spans = 5)
abline(0, 0)

# Spectral analysis for Y
spectrum(acc_data$Y, spans = 5)
abline(0, 0)

print(sd(acc_data$X))
print(sd(acc_data$Y))
