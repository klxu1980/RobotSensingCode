import math
import numpy as np
import matplotlib.pyplot as plt


def get_gauss(u, sig):
    x = np.linspace(u - 3 * sig, u + 3 * sig, 50)  # 定义域
    y = np.exp(-(x - u) ** 2 / (2 * sig ** 2)) / (math.sqrt(2 * math.pi) * sig)  # 定义曲线函数
    return x, y


x1, y1 = get_gauss(10, 1)
x2, y2 = get_gauss(1, 2)
x3, y3 = get_gauss(11, 2.23)

plt.plot(x1, y1, "g", linewidth=2)  # 加载曲线
plt.plot(x2, y2, "b", linewidth=2)  # 加载曲线
plt.plot(x3, y3, "r", linewidth=2)  # 加载曲线
plt.grid(True)  # 网格线
plt.show()

sig1 = 2.23
sig2 = 3
u1 = 11
u2 = 13
u = sig2**2 * u1 / (sig1**2 + sig2**2) + sig1**2 * u2 / (sig1**2 + sig2**2)
sig = math.sqrt(sig1**2 * sig2**2 / (sig1**2 + sig2**2))

x1, y1 = get_gauss(u1, sig1)
x2, y2 = get_gauss(u2, sig2)
x3, y3 = get_gauss(u, sig)

print("u1 = %.2f, sig1 = %.2f" % (u1, sig1))
print("u2 = %.2f, sig2 = %.2f" % (u2, sig2))
print("u3 = %.2f, sig3 = %.2f" % (u, sig))

plt.plot(x1, y1, "g", linewidth=2)  # 加载曲线
plt.plot(x2, y2, "b", linewidth=2)  # 加载曲线
plt.plot(x3, y3, "r", linewidth=2)  # 加载曲线
plt.grid(True)  # 网格线
plt.show()
